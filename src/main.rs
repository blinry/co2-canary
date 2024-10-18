#![no_std]
#![no_main]

mod display;
mod history;
mod sunrise;

use core::{
    ptr::{addr_of, addr_of_mut},
    time::Duration,
};
use display::Display;
use embedded_hal::delay::DelayNs;
use history::History;
use sunrise::{CalibrationData, SunriseSensor};

use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    delay::Delay,
    entry,
    gpio::{Input, Io, Level, NoPin, Output, Pull},
    i2c::I2c,
    prelude::*,
    rtc_cntl::{
        get_reset_reason, get_wakeup_cause,
        sleep::{RtcSleepConfig, TimerWakeupSource},
        Rtc, SocResetReason,
    },
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

    let mut temperature = 0.0;

    if true {
        let co2_enable = Output::new(io.pins.gpio25, Level::High);

        let sda = io.pins.gpio22;
        let scl = io.pins.gpio20;
        let i2c = I2c::new(peripherals.I2C0, sda, scl, 100u32.kHz());

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

                    CALIBRATION_DATA.update_time_ms(rtc.time_since_boot().ticks());

                    HISTORY.add_measurement(co2);
                println!("{:?}", CALIBRATION_DATA);
                }
                Err(e) => {
                    println!("Error: {:?}", e);
                }
            }
        }

        temperature = co2_sensor.get_temperature().unwrap();

        co2_sensor.turn_off();
    }

    // read battery voltage from GPIO pin 15
    let mut adc1_config = AdcConfig::new();
    let mut battery_voltage_pin =
        adc1_config.enable_pin(io.pins.gpio35, Attenuation::Attenuation11dB);
    //let analog = peripherals.SENS.split();
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);
    let battery_voltage = nb::block!(adc1.read_oneshot(&mut battery_voltage_pin)).unwrap() as f32;
    println!("Measured value: {}", battery_voltage);
    // No idea why 1.14 is a good factor here, but it matches my measurements at full battery level.
    // battery voltage 4.22 V <-> 1610 measured value
    // battery voltage 4.00 V <-> 1744 measured value
    let battery_voltage = battery_voltage * 1.16 * 2.0 / 1000.0;

    println!("Estimated battery voltage: {:.2} V\n", battery_voltage);

    if true {
        let sck = io.pins.gpio5;
        let mosi = io.pins.gpio19;
        let miso = io.pins.gpio21;

        let mut spi = Spi::new(peripherals.SPI3, 16u32.MHz(), SpiMode::Mode0)
            .with_pins(sck, mosi, miso, NoPin);

        let cs = Output::new(io.pins.gpio15, Level::High); // chip select
        let busy_in = Input::new(io.pins.gpio39, Pull::Down);
        let dc = Output::new(io.pins.gpio33, Level::Low); // data/command
        let rst = Output::new(io.pins.gpio26, Level::Low);

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
    // See https://learn.adafruit.com/adafruit-esp32-feather-v2/power-management
    let _neopixel_and_i2c_power = Output::new(io.pins.gpio2, Level::Low);
    //neopixel_and_i2c_power.set_low().unwrap();

    // Deep sleep.
    let mut delay = Delay::new();
    let timer = TimerWakeupSource::new(Duration::from_secs(55));
    println!("sleeping!");
    delay.delay_ms(100u32);

    let mut cfg = RtcSleepConfig::deep();
    cfg.set_rtc_fastmem_pd_en(false);
    rtc.sleep(&cfg, &[&timer]);
    panic!("We should never get here after the sleep() call.");
}
