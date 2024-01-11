#![no_std]
#![no_main]

use core::time::Duration;

use esp32_hal::{
    clock::ClockControl,
    entry,
    gpio::IO,
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::{get_reset_reason, get_wakeup_cause, sleep::TimerWakeupSource, SocResetReason},
    spi::{master::Spi, SpiMode},
    Cpu, Delay, Rtc,
};
use esp_backtrace as _;
use esp_println::println;

use embedded_graphics::{
    pixelcolor::BinaryColor::On as Black,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
};
use epd_waveshare::{epd2in9_v2::*, prelude::*};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    println!("up and runnning!");
    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("wake reason: {:?}", wake_reason);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // Pins for ESP32 Feather v2 + E-Ink Feather Wing
    let sck = io.pins.gpio5;
    let mosi = io.pins.gpio19;
    let miso = io.pins.gpio21;
    let cs = io.pins.gpio16; // ?
    let mut spi = Spi::new(peripherals.SPI3, 1000u32.kHz(), SpiMode::Mode0, &clocks).with_pins(
        Some(sck),
        Some(mosi),
        Some(miso),
        Some(cs),
    );

    let cs_pin = io.pins.gpio15.into_push_pull_output();
    let busy_in = io.pins.gpio27.into_pull_down_input(); // ?
    let dc = io.pins.gpio33.into_push_pull_output();
    let rst = io.pins.gpio12.into_push_pull_output(); // ?
    let mut delay = Delay::new(&clocks);

    let mut epd = Epd2in9::new(&mut spi, cs_pin, busy_in, dc, rst, &mut delay).unwrap();

    // Use display graphics from embedded-graphics
    let mut display = Display2in9::default();

    // Use embedded graphics for drawing a line

    let _ = Line::new(Point::new(0, 0), Point::new(20, 20))
        .into_styled(PrimitiveStyle::with_stroke(Black, 1))
        .draw(&mut display);

    // Display updated frame
    epd.update_frame(&mut spi, display.buffer(), &mut delay)
        .unwrap();
    epd.display_frame(&mut spi, &mut delay).unwrap();

    // Set the EPD to sleep
    epd.sleep(&mut spi, &mut delay).unwrap();

    let mut delay = Delay::new(&clocks);

    let timer = TimerWakeupSource::new(Duration::from_secs(10));
    println!("sleeping!");
    delay.delay_ms(100u32);
    rtc.sleep_deep(&[&timer], &mut delay);
}
