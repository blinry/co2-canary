use crate::history::History;
use core::fmt::Write;
use embedded_graphics::{
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
};
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::SpiDevice,
};
use epd_waveshare::{
    epd1in54_v2::{Display1in54, Epd1in54},
    prelude::*,
};
use heapless::String;
use itertools::Itertools;
use u8g2_fonts::{
    fonts,
    types::{FontColor, HorizontalAlignment, VerticalPosition},
    FontRenderer,
};

const CRITICAL_CO2: u16 = 1000;

pub struct Display<SPI, BUSY, DC, RST, DELAY> {
    epd: Epd1in54<SPI, BUSY, DC, RST, DELAY>,
    display: Display1in54,
    spi: SPI,
    delay: DELAY,
}

impl<SPI, BUSY, DC, RST, DELAY> Display<SPI, BUSY, DC, RST, DELAY>
where
    SPI: SpiDevice,
    DELAY: DelayNs,
    BUSY: InputPin,
    DC: OutputPin,
    RST: OutputPin,
{
    pub fn new(
        mut spi: SPI,
        busy_pin: BUSY,
        data_command_pin: DC,
        reset_pin: RST,
        mut delay: DELAY,
    ) -> Self {
        let epd = Epd1in54::new(
            &mut spi,
            busy_pin,
            data_command_pin,
            reset_pin,
            &mut delay,
            Some(5),
        )
        .unwrap();

        let mut display = Display1in54::default();
        display.set_rotation(DisplayRotation::Rotate270);
        display.clear(Color::White).unwrap();

        Display {
            epd,
            display,
            spi,
            delay,
        }
    }

    pub fn draw(
        &mut self,
        history: &History,
        temperature: f32,
        battery_percent: Option<f32>,
    ) -> Result<(), SPI::Error> {
        self.epd
            .set_lut(&mut self.spi, &mut self.delay, Some(RefreshLut::Full))?;

        self.draw_temperature(temperature);

        if let Some(battery_voltage) = battery_percent {
            self.draw_battery(battery_voltage);
        }

        if let Some(latest_co2) = history.recent() {
            self.draw_graph(history);
            self.draw_co2(latest_co2);
        }

        self.epd
            .update_and_display_frame(&mut self.spi, self.display.buffer(), &mut self.delay)?;

        self.epd.sleep(&mut self.spi, &mut self.delay)?;

        Ok(())
    }

    fn draw_co2(&mut self, co2: u16) {
        // TODO: Use the Results.
        let co2_font = FontRenderer::new::<fonts::u8g2_font_fub42_tr>();
        let mut co2_text = String::<32>::new();
        let _ = write!(&mut co2_text, "{co2}");

        let mut font_color = FontColor::Transparent(Color::Black);

        if co2 >= CRITICAL_CO2 {
            font_color = FontColor::Transparent(Color::White);

            // Draw black box under text.
            let rect = co2_font
                .get_rendered_dimensions_aligned(
                    co2_text.as_str(),
                    self.display.bounding_box().center() + Point::new(0, 5),
                    VerticalPosition::Baseline,
                    HorizontalAlignment::Center,
                )
                .expect("Should be able to look up all glyphs")
                .expect("Should result in a rectangle");

            let padding = 5u32;
            let _ = Rectangle::new(
                rect.top_left - Point::new(padding as i32, padding as i32),
                rect.size + Size::new(2 * padding, 2 * padding),
            )
            .into_styled(PrimitiveStyle::with_fill(Color::Black))
            .draw(&mut self.display);
        }

        co2_font
            .render_aligned(
                co2_text.as_str(),
                self.display.bounding_box().center() + Point::new(0, 5),
                VerticalPosition::Baseline,
                HorizontalAlignment::Center,
                font_color,
                &mut self.display,
            )
            .unwrap();

        let ppm_font = FontRenderer::new::<fonts::u8g2_font_fub25_tr>();
        let mut ppm_text = String::<32>::new();
        let _ = write!(&mut ppm_text, "ppm");
        ppm_font
            .render_aligned(
                ppm_text.as_str(),
                self.display.bounding_box().center() + Point::new(0, 35),
                VerticalPosition::Baseline,
                HorizontalAlignment::Center,
                FontColor::Transparent(Color::Black),
                &mut self.display,
            )
            .unwrap();
    }

    fn draw_temperature(&mut self, temperature: f32) {
        let temperature_font = FontRenderer::new::<fonts::u8g2_font_fub14_tr>();
        let mut temperature_text = String::<32>::new();
        let _ = write!(&mut temperature_text, "{temperature:.1} C");
        temperature_font
            .render_aligned(
                temperature_text.as_str(),
                Point::new(5, self.display.size().height as i32 - 5),
                VerticalPosition::Baseline,
                HorizontalAlignment::Left,
                FontColor::Transparent(Color::Black),
                &mut self.display,
            )
            .unwrap();
    }

    fn draw_battery(&mut self, percent: f32) {
        let battery_font = FontRenderer::new::<fonts::u8g2_font_fub14_tr>();
        let mut battery_text = String::<32>::new();
        let _ = write!(&mut battery_text, "{percent:.0}%");
        battery_font
            .render_aligned(
                battery_text.as_str(),
                Point::new(
                    self.display.size().width as i32 - 5,
                    self.display.size().height as i32 - 5,
                ),
                VerticalPosition::Baseline,
                HorizontalAlignment::Right,
                FontColor::Transparent(Color::Black),
                &mut self.display,
            )
            .unwrap();
    }

    fn draw_graph(&mut self, history: &History) {
        // Swapped because the display is rotated.
        let width = self.epd.height() as i32;
        let height = self.epd.width() as i32;

        // Find max value.
        let mut max_co2 = history.max_value().expect("No history to display");
        if max_co2 < CRITICAL_CO2 + 100 {
            max_co2 = CRITICAL_CO2 + 100;
        }

        let pixels_per_value = width as f32 / (History::max_size() - 1) as f32;

        for ((i1, v1), (i2, v2)) in history.iter().enumerate().tuple_windows() {
            let y1 = height - ((*v1 as i32) * height) / (max_co2 as i32);
            let y2 = height - ((*v2 as i32) * height) / (max_co2 as i32);

            let x1 = (i1 as f32 * pixels_per_value) as i32;
            let x2 = (i2 as f32 * pixels_per_value) as i32;

            let _ = Line::new(Point::new(x1, y1), Point::new(x2, y2))
                .into_styled(PrimitiveStyle::with_stroke(Color::Black, 2))
                .draw(&mut self.display);
        }

        // Draw dashed line at critical value.
        let y = height - ((CRITICAL_CO2 as i32) * height) / (max_co2 as i32);
        for x in (0..width).step_by(10) {
            let _ = Line::new(Point::new(x, y), Point::new(x + 5, y))
                .into_styled(PrimitiveStyle::with_stroke(Color::Black, 2))
                .draw(&mut self.display);
        }
    }
}
