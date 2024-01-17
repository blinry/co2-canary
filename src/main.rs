#![no_std]
#![no_main]

use core::time::Duration;

use esp32_hal::{
    clock::ClockControl,
    entry,
    gpio::{IO, NO_PIN},
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::sleep::TimerWakeupSource,
    spi::{master::Spi, SpiMode},
    Delay, Rtc,
};
use esp_backtrace as _;
use esp_println::println;

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::BinaryColor::On as Black,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
    text::Text,
};
use epd_waveshare::{epd2in9_v2::*, prelude::*};

use core::fmt::Write;
use heapless::String;

const SUNRISE_ADDR: u8 = 0x68;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    let mut co2 = 1337;

    if true {
        // Enable pin for the CO2 sensor.
        let mut co2_en = io.pins.gpio25.into_push_pull_output();
        co2_en.set_high().unwrap();

        let sda = io.pins.gpio22;
        let scl = io.pins.gpio20;
        let mut i2c = I2C::new(peripherals.I2C0, sda, scl, 100u32.kHz(), &clocks);

        delay.delay_ms(35u32);

        // Wake up the sensor. May be NACKed.
        let _ = i2c.write(SUNRISE_ADDR, &[]);

        delay.delay_ms(5u32);

        print_byte(&mut i2c, "measurement mode", 0x95);
        print_2_bytes(&mut i2c, "measurement period", 0x96);
        print_2_bytes(&mut i2c, "number of samples", 0x98);

        if get_byte(&mut i2c, 0x95) == 0 {
            println!("sensor is in continuous measurement mode, switching to single");
            i2c.write(SUNRISE_ADDR, &[0x95, 0x01]).unwrap();
        }

        // Start a measurement.
        i2c.write(SUNRISE_ADDR, &[0xC3, 0x01]).unwrap();

        // Light sleep for 3.4 seconds.
        let mut delay = Delay::new(&clocks);
        let timer = TimerWakeupSource::new(Duration::from_millis(3400));
        println!("light sleep!");
        delay.delay_ms(100u32);
        rtc.sleep_light(&[&timer], &mut delay);

        // Read the error from 0x00 and 0x01.
        let mut buf = [0u8; 2];
        i2c.write_read(SUNRISE_ADDR, &[0x00], &mut buf).unwrap();
        let error = u16::from_be_bytes(buf);
        println!("error (16 bits): {:016b}", error);

        // Read the product code from 0x70-0x7F (ASCII).
        let mut buf = [0u8; 16];
        i2c.write_read(SUNRISE_ADDR, &[0x70], &mut buf).unwrap();
        // print bytes in a loop
        for i in 0..16 {
            println!("{}", buf[i]);
        }

        // Read the CO2 concentration from 0x06 (MSB) and 0x07 (LSB).
        let mut buf = [0u8; 2];
        i2c.write_read(SUNRISE_ADDR, &[0x06], &mut buf).unwrap();

        // Set CO2 sensor to sleep.
        co2_en.set_low().unwrap();

        co2 = u16::from_be_bytes(buf);
        println!("CO2: {}", co2);
    }

    if true {
        let sck = io.pins.gpio5;
        let mosi = io.pins.gpio19;
        let miso = io.pins.gpio21;

        let mut spi = Spi::new(peripherals.SPI3, 4000u32.kHz(), SpiMode::Mode0, &clocks).with_pins(
            Some(sck),
            Some(mosi),
            Some(miso),
            NO_PIN,
        );

        let mut cs = io.pins.gpio15.into_push_pull_output(); // chip select
        cs.set_high().unwrap();
        let busy_in = io.pins.gpio39.into_pull_down_input();
        let dc = io.pins.gpio33.into_push_pull_output(); // data/command
        let rst = io.pins.gpio26.into_push_pull_output();
        let mut delay = Delay::new(&clocks);

        let mut epd = Epd2in9::new(&mut spi, cs, busy_in, dc, rst, &mut delay).unwrap();

        let mut display = Display2in9::default();

        let _ = Line::new(Point::new(0, 0), Point::new(20, 10))
            .into_styled(PrimitiveStyle::with_stroke(Black, 1))
            .draw(&mut display);

        let mut text = String::<32>::new();
        let _ = write!(&mut text, "CO2: {}", co2);
        let style = MonoTextStyle::new(&FONT_10X20, Black);
        let _ = Text::new(&text, Point::new(10, 30), style).draw(&mut display);

        epd.update_frame(&mut spi, display.buffer(), &mut delay)
            .unwrap();
        epd.display_frame(&mut spi, &mut delay).unwrap();
        println!("drawn!");

        epd.sleep(&mut spi, &mut delay).unwrap();
    }

    // Power off the neopixel and I2C bus, for low-power sleep.
    // See https://learn.adafruit.com/adafruit-esp32-feather-v2/power-management
    let mut neopixel_and_i2c_power = io.pins.gpio2.into_push_pull_output();
    neopixel_and_i2c_power.set_low().unwrap();

    // Deep sleep.
    let mut delay = Delay::new(&clocks);
    let timer = TimerWakeupSource::new(Duration::from_secs(60));
    println!("sleeping!");
    delay.delay_ms(100u32);
    rtc.sleep_deep(&[&timer], &mut delay);
}

fn get_byte<T>(i2c: &mut I2C<T>, reg: u8) -> u8
where
    T: _esp_hal_i2c_Instance,
{
    let mut buf = [0u8; 1];
    i2c.write_read(SUNRISE_ADDR, &[reg], &mut buf).unwrap();
    buf[0]
}

fn print_byte<T>(i2c: &mut I2C<T>, name: &str, reg: u8)
where
    T: _esp_hal_i2c_Instance,
{
    println!("{}: {}", name, get_byte(i2c, reg));
}

fn print_2_bytes<T>(i2c: &mut I2C<T>, name: &str, reg: u8)
where
    T: _esp_hal_i2c_Instance,
{
    let mut buf = [0u8; 2];
    i2c.write_read(SUNRISE_ADDR, &[reg], &mut buf).unwrap();
    let value = u16::from_be_bytes(buf);
    println!("{}: {}", name, value);
}
