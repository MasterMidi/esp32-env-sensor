#![allow(dead_code)]

use embedded_hal::blocking::i2c::*;

pub const ADDRESS: u8 = 0x12;

pub struct Pmsa003i<T>
where
    T: WriteRead + Read + Write,
{
    i2c: T,
    address: u8,
}

impl<T> Pmsa003i<T>
where
    T: WriteRead + Read + Write,
{
    /// Creates a new sensor driver.
    pub fn new(i2c: T, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Releases the underlying I2C bus and destroys the driver.
    pub fn release(self) -> T {
        self.i2c
    }
}

impl<I2C, E> Pmsa003i<I2C>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    E: std::fmt::Debug,
{
    pub fn read(&mut self) -> Result<Reading, SensorError<E>> {
        let mut buf: [u8; PAYLOAD_LEN] = [0; PAYLOAD_LEN];
        self.i2c.read(self.address, &mut buf)?;
        if buf[0] != MAGIC_BYTE_0 || buf[1] != MAGIC_BYTE_1 {
            Err(SensorError::BadMagic)
        } else {
            parse_data(&buf)
        }
    }
}

/// A single air quality sensor reading
#[derive(Debug, Clone, Copy)]
pub struct Reading {
    pm1: u16,
    pm2_5: u16,
    pm10: u16,
    env_pm1: u16,
    env_pm2_5: u16,
    env_pm10: u16,
    particles_0_3: u16,
    particles_0_5: u16,
    particles_1: u16,
    particles_2_5: u16,
    particles_5: u16,
    particles_10: u16,
}

impl Reading {
    /// Returns the standard PM1 concentration in µg/m³
    pub fn pm1(&self) -> u16 {
        self.pm1
    }

    /// Returns the standard PM2.5 concentration in µg/m³
    pub fn pm2_5(&self) -> u16 {
        self.pm2_5
    }

    /// Returns the standard PM10 concentration in µg/m³
    pub fn pm10(&self) -> u16 {
        self.pm10
    }

    /// Returns the environmental PM1 concentration in µg/m³
    ///
    /// Note that some devices do not support this reading and will
    /// return garbage data for this value.
    pub fn env_pm1(&self) -> u16 {
        self.env_pm1
    }

    /// Returns the environmental PM2.5 concentration in µg/m³
    ///
    /// Note that some devices do not support this reading and will
    /// return garbage data for this value.
    pub fn env_pm2_5(&self) -> u16 {
        self.env_pm2_5
    }

    /// Returns the environmental PM10 concentration in µg/m³
    ///
    /// Note that some devices do not support this reading and will
    /// return garbage data for this value.
    pub fn env_pm10(&self) -> u16 {
        self.env_pm10
    }

    /// Returns count of particles smaller than 0.3µm
    pub fn particles_0_3(&self) -> u16 {
        self.particles_0_3
    }

    /// Returns count of particles smaller than 0.5µm
    pub fn particles_0_5(&self) -> u16 {
        self.particles_0_5
    }

    /// Returns count of particles smaller than 1µm
    pub fn particles_1(&self) -> u16 {
        self.particles_1
    }

    /// Returns count of particles smaller than 2.5µm
    pub fn particles_2_5(&self) -> u16 {
        self.particles_2_5
    }

    /// Returns count of particles smaller than 5µm
    pub fn particles_5(&self) -> u16 {
        self.particles_5
    }

    /// Returns count of particles smaller than 10µm
    pub fn particles_10(&self) -> u16 {
        self.particles_10
    }
}

const MAGIC_BYTE_0: u8 = 0x42;
const MAGIC_BYTE_1: u8 = 0x4d;
const PAYLOAD_LEN: usize = 32;

fn parse_data<E: std::fmt::Debug>(buf: &[u8; PAYLOAD_LEN]) -> Result<Reading, SensorError<E>> {
    let sum = buf[0..PAYLOAD_LEN - 2]
        .iter()
        .fold(0u16, |accum, next| accum + *next as u16);
    let expected_sum: u16 = ((buf[PAYLOAD_LEN - 2] as u16) << 8) | (buf[PAYLOAD_LEN - 1] as u16);
    if expected_sum == sum {
        Ok(Reading {
            pm1: as_u16(buf[4], buf[5]),
            pm2_5: as_u16(buf[6], buf[7]),
            pm10: as_u16(buf[8], buf[9]),
            env_pm1: as_u16(buf[10], buf[11]),
            env_pm2_5: as_u16(buf[12], buf[13]),
            env_pm10: as_u16(buf[14], buf[15]),
            particles_0_3: as_u16(buf[16], buf[17]),
            particles_0_5: as_u16(buf[18], buf[19]),
            particles_1: as_u16(buf[20], buf[21]),
            particles_2_5: as_u16(buf[22], buf[23]),
            particles_5: as_u16(buf[24], buf[25]),
            particles_10: as_u16(buf[26], buf[27]),
        })
    } else {
        Err(SensorError::ChecksumMismatch)
    }
}

fn as_u16(hi: u8, lo: u8) -> u16 {
    ((hi as u16) << 8) | (lo as u16)
}

/// Describes errors returned by the air quality sensor
#[derive(Debug)]
pub enum SensorError<E: std::fmt::Debug> {
    /// Couldn't find the "magic" bytes that indicate the start of a data frame
    ///
    /// This likely means that you've set an incorrect baud rate, or there is something
    /// noisy about your connection to the device.
    BadMagic,
    /// The checksum provided in the sensor data did not match the checksum of the data itself
    ///
    /// Retrying the read will usually clear up the problem.
    ChecksumMismatch,
    /// Read error from the serial device or I2C bus
    ReadError(E),
}

impl<E: std::fmt::Debug> std::fmt::Display for SensorError<E> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        use SensorError::*;
        match self {
            BadMagic => f.write_str("Unable to find magic bytes at start of payload"),
            ChecksumMismatch => f.write_str("Data read was corrupt"),
            ReadError(error) => write!(f, "Read error: {:?}", error),
        }
    }
}

#[cfg(feature = "std")]
impl<E: std::fmt::Debug> std::error::Error for SensorError<E> {}

impl<E: std::fmt::Debug> From<E> for SensorError<E> {
    fn from(error: E) -> Self {
        SensorError::ReadError(error)
    }
}
