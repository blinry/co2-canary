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

const SUNRISE_ADDR: u8 = 0x68;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    /*
    let sda = io.pins.gpio22;
    let scl = io.pins.gpio20;

    // Datasheet https://rmtplusstoragesenseair.blob.core.windows.net/docs/Dev/publicerat/TDE5531.pdf
    // says "100 kbit/s".
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
    // Wait for 3.4 seconds.
    delay.delay_ms(3400u32);

    // Read the error from 0x00 and 0x01.
    let mut buf = [0u8; 2];
    i2c.write_read(SUNRISE_ADDR, &[0x00], &mut buf).unwrap();
    let error = u16::from_be_bytes(buf);
    println!("error in binary (16 bits): {:016b}", error);

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
    let co2 = u16::from_be_bytes(buf);
    println!("CO2: {}", co2);
    */

    /*


    println!("up and runnning!");
    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);
    */

    // Pins for ESP32 Feather v2 + E-Ink Feather Wing

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

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
    let busy_in = io.pins.gpio26.into_pull_down_input(); // ?
    let dc = io.pins.gpio33.into_push_pull_output(); // data/command
    let rst = io.pins.gpio13.into_push_pull_output(); // ?
    let mut delay = Delay::new(&clocks);

    let mut epd = Epd2in9::new(&mut spi, cs, busy_in, dc, rst, &mut delay).unwrap();

    // Use display graphics from embedded-graphics
    let mut display = Display2in9::default();

    //epd.wake_up(&mut spi, &mut delay).unwrap();
    //epd.clear_frame(&mut spi, &mut delay).unwrap();
    //epd.display_frame(&mut spi, &mut delay).unwrap();

    //println!("cleared!");
    //delay.delay_ms(2000u32);
    //println!("waited!");

    // Use embedded graphics for drawing a line
    let _ = Line::new(Point::new(0, 0), Point::new(20, 10))
        .into_styled(PrimitiveStyle::with_stroke(Black, 1))
        .draw(&mut display);

    let text = "ha this does work";
    let style = MonoTextStyle::new(&FONT_10X20, Black);
    let _ = Text::new(text, Point::new(30, 30), style).draw(&mut display);

    epd.update_frame(&mut spi, display.buffer(), &mut delay)
        .unwrap();
    epd.display_frame(&mut spi, &mut delay).unwrap();
    println!("drawn!");
    delay.delay_ms(1000u32);
    println!("waited!");

    // Set the EPD to sleep
    epd.sleep(&mut spi, &mut delay).unwrap();

    let mut delay = Delay::new(&clocks);
    let timer = TimerWakeupSource::new(Duration::from_secs(10));
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
