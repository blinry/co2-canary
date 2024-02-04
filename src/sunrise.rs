use core::time::Duration;
use embedded_hal::{delay::DelayNs, digital::OutputPin, i2c::I2c};
use esp_println::println;

/// See "I2C on Senseair Sunrise & Sunlight" document at https://senseair.com/product/sunrise/

const SUNRISE_ADDR: u8 = 0x68;

#[derive(Debug)]
pub struct CalibrationData {
    abc_data: [u8; 10],
    iir_data: [u8; 14],
    last_timestamp_ms: Option<u64>,
}

impl CalibrationData {
    pub const fn new() -> Self {
        CalibrationData {
            abc_data: [0; 10],
            iir_data: [0; 14],
            last_timestamp_ms: None,
        }
    }
    pub fn update_time_ms(&mut self, ms: u64) {
        if let Some(last_ms) = self.last_timestamp_ms {
            let elapsed_ms = ms - last_ms;
            let duration = Duration::from_millis(elapsed_ms);
            let elapsed_hours = duration.as_secs() / 3600;
            self.increment_hour(elapsed_hours as u16);
        }
        self.last_timestamp_ms = Some(ms);
    }
    pub fn increment_hour(&mut self, hours: u16) {
        let mut hour = u16::from_be_bytes([self.abc_data[0], self.abc_data[1]]);
        hour += hours;
        let buf = hour.to_be_bytes();
        self.abc_data[0] = buf[0];
        self.abc_data[1] = buf[1];
    }
}

pub struct SunriseSensor<I2C, ENABLE, DELAY> {
    enable_pin: ENABLE,
    i2c: I2C,
    delay: DELAY,
}

impl<I2C, ENABLE, DELAY> SunriseSensor<I2C, ENABLE, DELAY>
where
    I2C: I2c,
    ENABLE: OutputPin,
    DELAY: DelayNs,
{
    pub fn new(i2c: I2C, enable_pin: ENABLE, delay: DELAY) -> Self {
        SunriseSensor {
            enable_pin,
            i2c,
            delay,
        }
    }

    pub fn init(&mut self, number_of_samples: u16) -> Result<(), I2C::Error> {
        self.enable_pin.set_high().unwrap();

        // Manual says to wait 35ms after power-on.
        self.delay.delay_ms(35u32);

        // Wake up the sensor. May be NACKed.
        let _ = self.i2c.write(SUNRISE_ADDR, &[]);
        self.delay.delay_ms(5u32);

        let single_measurement_mode = 1;
        self.set_2_bytes("measurement mode", 0x95, single_measurement_mode)?;

        self.set_2_bytes("number of samples", 0x98, number_of_samples)?;

        let mut buf = [0u8; 1];
        self.i2c.write_read(SUNRISE_ADDR, &[0x81], &mut buf)?;
        println!("calibration status: {:08b}", buf[0]);

        let abc_period_hours = 20;
        self.set_2_bytes("ABC period", 0x9A, abc_period_hours)?;

        let co2_baseline_ppm = 417;
        self.set_2_bytes("ABC target", 0x9E, co2_baseline_ppm)?;

        let mut meter_control = [0u8];
        self.i2c
            .write_read(SUNRISE_ADDR, &[0xA5], &mut meter_control)?;
        println!("meter control: {:08b}", meter_control[0]);
        if meter_control[0] & 0b0000_0010 != 0 {
            // ABC enable bit is not set to 0 (which enables it).
            // Let's change that.
            println!("enabling ABC");
            meter_control[0] &= 0b1111_1101;
            self.i2c.write(SUNRISE_ADDR, &[0xA5, meter_control[0]])?;
        }

        Ok(())
    }

    // TODO: Return a WakeupSource?
    pub fn start_measurement(
        &mut self,
        calibration_data: Option<&CalibrationData>,
    ) -> Result<(), I2C::Error> {
        let mut buf: [u8; 26] = [0; 26];

        buf[0] = 0xC3;
        buf[1] = 0x1;

        // If we have calibration data, use it.
        if let Some(calibration_data) = calibration_data {
            buf[2..12].copy_from_slice(&calibration_data.abc_data);
            buf[12..26].copy_from_slice(&calibration_data.iir_data);
        }

        // Start a measurement.
        self.i2c.write(SUNRISE_ADDR, &buf)?;

        Ok(())
    }

    pub fn get_co2(&mut self, calibration_data: &mut CalibrationData) -> Result<u16, I2C::Error> {
        //// Read the error from 0x00 and 0x01.
        //let mut buf = [0u8; 2];
        //self.i2c.write_read(SUNRISE_ADDR, &[0x00], &mut buf)?;
        //let error = u16::from_be_bytes(buf);
        //println!("error (16 bits): {:016b}", error);

        //// Read the product code from 0x70-0x7F (ASCII).
        //let mut buf = [0u8; 16];
        //self.i2c.write_read(SUNRISE_ADDR, &[0x70], &mut buf)?;
        //// print bytes in a loop
        //for i in 0..16 {
        //    println!("{}", buf[i]);
        //}

        // Read the CO2 concentration from 0x06 (MSB) and 0x07 (LSB).
        let co2 = self.get_2_bytes(0x06)?;

        let mut abc_and_iir_data: [u8; 24] = [0; 24];
        self.i2c
            .write_read(SUNRISE_ADDR, &[0xC4], &mut abc_and_iir_data)?;

        calibration_data
            .abc_data
            .copy_from_slice(&abc_and_iir_data[0..10]);
        calibration_data
            .iir_data
            .copy_from_slice(&abc_and_iir_data[10..24]);

        Ok(co2)
    }

    pub fn get_temperature(&mut self) -> Result<f32, I2C::Error> {
        let temperature = self.get_2_bytes(0x08)?;
        Ok((temperature as f32) / 100.0)
    }

    pub fn turn_off(&mut self) {
        // Set CO2 sensor to sleep.
        // TODO: Use the Result?
        let _ = self.enable_pin.set_low();
    }

    fn get_byte(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(SUNRISE_ADDR, &[reg], &mut buf)?;
        Ok(buf[0])
    }

    fn get_2_bytes(&mut self, reg: u8) -> Result<u16, I2C::Error> {
        let mut buf = [0u8; 2];
        self.i2c.write_read(SUNRISE_ADDR, &[reg], &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }

    fn print_byte(&mut self, name: &str, reg: u8) -> Result<(), I2C::Error> {
        println!("{}: {}", name, self.get_byte(reg)?);
        Ok(())
    }

    fn print_2_bytes(&mut self, name: &str, reg: u8) {
        let mut buf = [0u8; 2];
        self.i2c.write_read(SUNRISE_ADDR, &[reg], &mut buf).unwrap();
        let value = u16::from_be_bytes(buf);
        println!("{}: {}", name, value);
    }

    fn set_byte(&mut self, name: &str, reg: u8, value: u8) -> Result<(), I2C::Error> {
        let current_value = self.get_byte(reg)?;
        if current_value == value {
            println!("{} is already set to {}", name, value);
        } else {
            println!("setting {} to {}", name, value);
            self.i2c.write(SUNRISE_ADDR, &[reg, value])?;
            let current_value = self.get_byte(reg)?;
            println!("{} is now set to {}", name, current_value);
        }
        Ok(())
    }

    fn set_2_bytes(&mut self, name: &str, reg: u8, value: u16) -> Result<(), I2C::Error> {
        let current_value = self.get_2_bytes(reg)?;
        if current_value == value {
            println!("{} is already set to {}", name, value);
        } else {
            println!("setting {} to {}", name, value);
            let buf = value.to_be_bytes();
            self.i2c.write(SUNRISE_ADDR, &[reg, buf[0], buf[1]])?;
            let current_value = self.get_2_bytes(reg)?;
            println!("{} is now set to {}", name, current_value);
        }
        Ok(())
    }
}
