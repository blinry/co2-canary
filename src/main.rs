#![no_std]
#![no_main]

mod display;
mod history;
mod sunrise;

use core::time::Duration;
use display::Display;
use history::History;
use sunrise::{CalibrationData, SunriseSensor};

use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    entry,
    gpio::{Input, Io, Level, NoPin, Output, Pull},
    i2c::I2c,
    prelude::*,
    rtc_cntl::{get_reset_reason, get_wakeup_cause, sleep::TimerWakeupSource, Rtc, SocResetReason},
    spi::{master::Spi, SpiMode},
    Cpu,
};
use esp_println::println;

#[ram(rtc_fast)]
static mut HISTORY: History = History::new();

#[ram(rtc_fast)]
static mut CALIBRATION_DATA: CalibrationData = CalibrationData::new();

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut delay = Delay::new();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rtc = Rtc::new(peripherals.LPWR);

    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let mut neopixel_and_i2c_power = Output::new(io.pins.gpio20, Level::Low);

    let mut temperature = 0.0;

    // Required for I2C to work!
    neopixel_and_i2c_power.set_high();

    let sda = io.pins.gpio19;
    let scl = io.pins.gpio18;
    let mut i2c = I2c::new(peripherals.I2C0, sda, scl, 100.kHz());

    if true {
        let co2_enable = Output::new(io.pins.gpio3, Level::High);

        let number_of_samples = 2;
        let mut co2_sensor = SunriseSensor::new(i2c, co2_enable, &mut delay);
        co2_sensor
            .init(number_of_samples)
            .expect("Could not initialize CO2 sensor");

        unsafe {
            co2_sensor
                .start_measurement(Some(&CALIBRATION_DATA))
                .expect("Could not start CO2 measurement");
        }

        // TODO: I can probably lower this?
        let milliseconds_per_sample = 300;
        let timer = TimerWakeupSource::new(Duration::from_millis(
            (number_of_samples * milliseconds_per_sample) as u64,
        ));
        rtc.sleep_light(&[&timer]);

        unsafe {
            match co2_sensor.get_co2(&mut CALIBRATION_DATA) {
                Ok(co2) => {
                    println!("CO2: {} ppm", co2);
                    HISTORY.add_measurement(co2);
                }
                Err(e) => {
                    println!("Error: {:?}", e);
                    HISTORY.add_measurement(0);
                }
            }
            CALIBRATION_DATA.update_time_ms(rtc.time_since_boot().ticks() / 1000);
            println!("{:?}", CALIBRATION_DATA);
        }

        temperature = co2_sensor.get_temperature().unwrap();

        co2_sensor.turn_off();
        i2c = co2_sensor.release();
    }

    let battery_voltage = {
        let mut battery = max17048::Max17048::new(i2c, 0x36);
        battery.soc().ok().map(|x| x as f32)
    };

    if true {
        let sck = io.pins.gpio21;
        let mosi = io.pins.gpio22;
        let miso = io.pins.gpio23;

        let mut spi = Spi::new(peripherals.SPI2, 16u32.MHz(), SpiMode::Mode0)
            .with_pins(sck, mosi, miso, NoPin);

        let cs = Output::new(io.pins.gpio7, Level::High); // chip select
        let busy_in = Input::new(io.pins.gpio5, Pull::Down);
        let dc = Output::new(io.pins.gpio8, Level::Low); // data/command
        let rst = Output::new(io.pins.gpio1, Level::Low);

        let exclusive_spi = ExclusiveDevice::new(&mut spi, cs, &mut delay)
            .expect("Failed to get exclusive SPI device");
        let mut delay = Delay::new();

        let mut display = Display::new(exclusive_spi, busy_in, dc, rst, &mut delay);

        unsafe {
            display
                .draw(&HISTORY, temperature, battery_voltage)
                .expect("Failed to draw to the display");
        }
    }

    // Power off the neopixel and I2C bus, for low-power sleep.
    // See https://learn.adafruit.com/adafruit-esp32-c6-feather/low-power-use
    neopixel_and_i2c_power.set_low();

    // Deep sleep.
    let timer = TimerWakeupSource::new(Duration::from_secs(55));
    rtc.sleep_deep(&[&timer]);
}
