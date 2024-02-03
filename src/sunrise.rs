use embedded_hal::{delay::DelayNs, digital::OutputPin, i2c::I2c};
use esp_println::println;

const SUNRISE_ADDR: u8 = 0x68;

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
        let mut sensor = SunriseSensor {
            i2c,
            enable_pin,
            delay,
        };

        // TODO: Use the Result?
        let _ = sensor.init();

        sensor
    }

    pub fn init(&mut self) -> Result<(), I2C::Error> {
        self.enable_pin.set_high().unwrap();

        // Manual says to wait 35ms after power-on.
        self.delay.delay_ms(35u32);

        // Wake up the sensor. May be NACKed.
        let _ = self.i2c.write(SUNRISE_ADDR, &[]);
        self.delay.delay_ms(5u32);

        self.print_byte("measurement mode", 0x95)?;
        self.print_2_bytes("measurement period", 0x96);
        self.print_2_bytes("number of samples", 0x98);

        if self.get_byte(0x95).unwrap() == 0 {
            println!("sensor is in continuous measurement mode, switching to single");
            self.i2c.write(SUNRISE_ADDR, &[0x95, 0x01])?
        }

        Ok(())
    }

    pub fn get_co2(&mut self) -> Result<u16, I2C::Error> {
        // Start a measurement.
        self.i2c.write(SUNRISE_ADDR, &[0xC3, 0x01])?;

        // Sleep for 3.4 seconds.
        // TODO: Caller might want to light sleep.
        self.delay.delay_ms(3400u32);

        // Read the error from 0x00 and 0x01.
        let mut buf = [0u8; 2];
        self.i2c.write_read(SUNRISE_ADDR, &[0x00], &mut buf)?;
        let error = u16::from_be_bytes(buf);
        println!("error (16 bits): {:016b}", error);

        // Read the product code from 0x70-0x7F (ASCII).
        let mut buf = [0u8; 16];
        self.i2c.write_read(SUNRISE_ADDR, &[0x70], &mut buf)?;
        // print bytes in a loop
        for i in 0..16 {
            println!("{}", buf[i]);
        }

        // Read the CO2 concentration from 0x06 (MSB) and 0x07 (LSB).
        let mut buf = [0u8; 2];
        self.i2c.write_read(SUNRISE_ADDR, &[0x06], &mut buf)?;

        // Set CO2 sensor to sleep.
        // TODO: Use the Result?
        let _ = self.enable_pin.set_low();

        Ok(u16::from_be_bytes(buf))
    }

    fn get_byte(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(SUNRISE_ADDR, &[reg], &mut buf)?;
        Ok(buf[0])
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
}
