#![no_std]
#![no_main]
// Allow this for now, to be able to use ram(rtc_fast).
#![allow(static_mut_refs)]

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
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    ram,
    rtc_cntl::{sleep::TimerWakeupSource, Rtc},
    spi::master::{Config as SpiConfig, Spi},
    time::{Instant, Rate},
};
use esp_println::println;

#[ram(rtc_fast)]
static mut HISTORY: History = History::new();

#[ram(rtc_fast)]
static mut CALIBRATION_DATA: CalibrationData = CalibrationData::new();

#[ram(rtc_fast)]
static mut LAST_DISPLAYED_CO2: u16 = 0;

#[esp_hal::main]
fn main() -> ! {
    let wakeup_time = Instant::now();

    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut delay = Delay::new();
    let mut rtc = Rtc::new(peripherals.LPWR);

    let mut neopixel_and_i2c_power =
        Output::new(peripherals.GPIO20, Level::Low, OutputConfig::default());

    let mut temperature = 0.0;

    // Required for I2C to work!
    neopixel_and_i2c_power.set_high();

    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(100));
    let mut i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("Should be able to configure I2C peripheral")
        .with_sda(peripherals.GPIO19)
        .with_scl(peripherals.GPIO18);

    if true {
        let co2_enable = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());

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
        let milliseconds_per_sample = 300u64;
        let timer = TimerWakeupSource::new(Duration::from_millis(
            (number_of_samples as u64) * milliseconds_per_sample,
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
            CALIBRATION_DATA.update_time_ms(rtc.time_since_boot().as_millis());
            println!("{:?}", CALIBRATION_DATA);
        }

        temperature = co2_sensor.get_temperature().unwrap();

        co2_sensor.turn_off();
        i2c = co2_sensor.release();
    }

    let battery_percent = max170xx::Max17048::new(i2c).soc().ok();

    // Refresh the display if the CO2 value has changed by more than a certain amount.
    let refresh_threshold = 50; // ppm
    let refresh_display =
        unsafe { HISTORY.recent().unwrap_or(0).abs_diff(LAST_DISPLAYED_CO2) >= refresh_threshold };

    if refresh_display {
        let mut spi = Spi::new(peripherals.SPI2, SpiConfig::default())
            .expect("Should be able to configure SPI device")
            .with_sck(peripherals.GPIO21)
            .with_mosi(peripherals.GPIO22)
            .with_miso(peripherals.GPIO23);

        let cs = Output::new(peripherals.GPIO7, Level::High, OutputConfig::default()); // chip select
        let busy_in = Input::new(
            peripherals.GPIO5,
            InputConfig::default().with_pull(Pull::Down),
        );
        let dc = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default()); // data/command
        let rst = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());

        let exclusive_spi = ExclusiveDevice::new(&mut spi, cs, &mut delay)
            .expect("Failed to get exclusive SPI device");
        let mut delay = Delay::new();

        let mut display = Display::new(exclusive_spi, busy_in, dc, rst, &mut delay);

        unsafe {
            display
                .draw(&HISTORY, temperature, battery_percent)
                .expect("Failed to draw to the display");
            LAST_DISPLAYED_CO2 = HISTORY.recent().unwrap_or(0);
        }
    }

    // Power off the neopixel and I2C bus, for low-power sleep.
    // See https://learn.adafruit.com/adafruit-esp32-c6-feather/low-power-use
    neopixel_and_i2c_power.set_low();

    // Deep sleep.
    let wakeup_interval = Duration::from_secs(30);
    let awake_duration = Instant::now() - wakeup_time;
    // (Convert to std Duration.)
    let awake_duration = Duration::from_millis(awake_duration.as_millis());
    let remaining_time = wakeup_interval - awake_duration;
    let timer = TimerWakeupSource::new(remaining_time);
    rtc.sleep_deep(&[&timer]);
}
