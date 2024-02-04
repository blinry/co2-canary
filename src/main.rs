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
use esp32_hal::{
    clock::ClockControl,
    entry,
    gpio::{IO, NO_PIN},
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::{
        get_reset_reason, get_wakeup_cause,
        sleep::{RtcSleepConfig, TimerWakeupSource},
        SocResetReason,
    },
    spi::{master::Spi, SpiMode},
    Cpu, Delay, Rtc,
};
use esp_backtrace as _;
use esp_println::println;

#[ram(rtc_fast)]
static mut HISTORY: History = History::new();

#[ram(rtc_fast)]
static mut CALIBRATION_DATA: CalibrationData = CalibrationData::new();

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rtc = Rtc::new(peripherals.LPWR);

    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let mut temperature = 0.0;

    if true {
        let co2_enable = io.pins.gpio25.into_push_pull_output();

        let sda = io.pins.gpio22;
        let scl = io.pins.gpio20;
        let i2c = I2C::new(peripherals.I2C0, sda, scl, 100u32.kHz(), &clocks);

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

        let mut delay = Delay::new(&clocks);
        let milliseconds_per_sample = 300;
        let timer = TimerWakeupSource::new(Duration::from_millis(
            (number_of_samples * milliseconds_per_sample) as u64,
        ));
        rtc.sleep_light(&[&timer], &mut delay);

        unsafe {
            let co2 = co2_sensor.get_co2(&mut CALIBRATION_DATA).unwrap();
            println!("CO2: {} ppm", co2);

            CALIBRATION_DATA.update_time_ms(rtc.get_time_ms());

            HISTORY.add_measurement(co2);
            println!("{:?}", CALIBRATION_DATA);
        }

        temperature = co2_sensor.get_temperature().unwrap();

        co2_sensor.turn_off();
    }

    if true {
        let sck = io.pins.gpio5;
        let mosi = io.pins.gpio19;
        let miso = io.pins.gpio21;

        let mut spi = Spi::new(peripherals.SPI3, 16u32.MHz(), SpiMode::Mode0, &clocks).with_pins(
            Some(sck),
            Some(mosi),
            Some(miso),
            NO_PIN,
        );

        let cs = io.pins.gpio15.into_push_pull_output(); // chip select
        let busy_in = io.pins.gpio39.into_pull_down_input();
        let dc = io.pins.gpio33.into_push_pull_output(); // data/command
        let rst = io.pins.gpio26.into_push_pull_output();

        let exclusive_spi = ExclusiveDevice::new(&mut spi, cs, &mut delay);
        let mut delay = Delay::new(&clocks);

        let mut display = Display::new(exclusive_spi, busy_in, dc, rst, &mut delay);

        unsafe {
            display
                .draw(HISTORY.data_for_display(), temperature)
                .expect("Failed to draw to the display");
        }
    }

    // Power off the neopixel and I2C bus, for low-power sleep.
    // See https://learn.adafruit.com/adafruit-esp32-feather-v2/power-management
    let mut neopixel_and_i2c_power = io.pins.gpio2.into_push_pull_output();
    neopixel_and_i2c_power.set_low().unwrap();

    // Deep sleep.
    let mut delay = Delay::new(&clocks);
    let timer = TimerWakeupSource::new(Duration::from_secs(55));
    println!("sleeping!");
    delay.delay_ms(100u32);

    let mut cfg = RtcSleepConfig::deep();
    cfg.set_rtc_fastmem_pd_en(false);
    rtc.sleep(&cfg, &[&timer], &mut delay);
    panic!("We should never get here after the sleep() call.");
}
