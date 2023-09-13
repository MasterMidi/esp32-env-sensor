#![allow(dead_code)]

use core::{
    convert::TryFrom,
    ops::{Deref, DerefMut},
};

use serde::Serialize;

use embedded_hal::blocking::i2c::*;

use bitfield::bitfield;

pub const ADDRESS: u8 = 0x53;

// ENS160 Register address
// This 2-byte register contains the part number in little endian of the ENS160.
const ENS160_PART_ID_REG: u8 = 0x00;
// This 1-byte register sets the Operating Mode of the ENS160.
const ENS160_OPMODE_REG: u8 = 0x10;
// This 1-byte register configures the action of the INTn pin.
const ENS160_CONFIG_REG: u8 = 0x11;
// This 1-byte register allows some additional commands to be executed on the ENS160.
#[allow(dead_code)]
const ENS160_COMMAND_REG: u8 = 0x12;
// This 2-byte register allows the host system to write ambient temperature data to ENS160 for compensation.
const ENS160_TEMP_IN_REG: u8 = 0x13;
// This 2-byte register allows the host system to write relative humidity data to ENS160 for compensation.
#[allow(dead_code)]
const ENS160_RH_IN_REG: u8 = 0x15;
// This 1-byte register indicates the current STATUS of the ENS160.
const ENS160_DATA_STATUS_REG: u8 = 0x20;
// This 1-byte register reports the calculated Air Quality Index according to the UBA.
const ENS160_DATA_AQI_REG: u8 = 0x21;
// This 2-byte register reports the calculated TVOC concentration in ppb.
const ENS160_DATA_TVOC_REG: u8 = 0x22;
// This 2-byte register reports the calculated equivalent CO2-concentration in ppm, based on the detected VOCs and hydrogen.
const ENS160_DATA_ECO2_REG: u8 = 0x24;
// This 2-byte register reports the temperature used in its calculations (taken from TEMP_IN, if supplied).
const ENS160_DATA_T_REG: u8 = 0x30;
// This 2-byte register reports the relative humidity used in its calculations (taken from RH_IN if supplied).
#[allow(dead_code)]
const ENS160_DATA_RH_REG: u8 = 0x32;
// This 1-byte register reports the calculated checksum of the previous DATA_ read transaction (of n-bytes).
#[allow(dead_code)]
const ENS160_DATA_MISR_REG: u8 = 0x38;
// This 8-byte register is used by several functions for the Host System to pass data to the ENS160.
#[allow(dead_code)]
const ENS160_GPR_WRITE_REG: u8 = 0x40;
// This 8-byte register is used by several functions for the ENS160 to pass data to the Host System.
#[allow(dead_code)]
const ENS160_GPR_READ_REG: u8 = 0x48;

pub struct Ens160<T>
where
    T: WriteRead + Read + Write,
{
    i2c: T,
    address: u8,
}

impl<T> Ens160<T>
where
    T: WriteRead + Read + Write,
{
    /// Creates a new sensor driver.
    pub fn new(i2c: T, address: u8) -> Ens160<T> {
        Self { i2c, address }
    }

    /// Releases the underlying I2C bus and destroys the driver.
    pub fn release(self) -> T {
        self.i2c
    }
}

impl<I2C, E> Ens160<I2C>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
    /// Resets the device.
    pub fn reset(&mut self) -> Result<(), E> {
        self.write_register([ENS160_OPMODE_REG, OperationMode::Reset as u8])
    }

    /// Switches the device to operational mode.
    ///
    /// Call this function when you want the device to start taking measurements.
    pub fn operational(&mut self) -> Result<(), E> {
        self.write_register([ENS160_OPMODE_REG, OperationMode::Standard as u8])
    }

    /// Returns the current status of the sensor.
    pub fn status(&mut self) -> Result<Status, E> {
        self.read_register::<1>(ENS160_DATA_STATUS_REG)
            .map(|v| Status(v[0]))
    }

    /// Returns the current Air Quality Index (AQI) reading from the sensor.
    ///
    /// The AQI is calculated based on the current sensor readings.
    pub fn airquality_index(&mut self) -> Result<AirqualityIndex, E> {
        self.read_register::<1>(ENS160_DATA_AQI_REG)
            .map(|v| AirqualityIndex::from(v[0] & 0x07))
    }

    /// Returns the Total Volatile Organic Compounds (TVOC) measurement from the sensor.
    ///
    /// The TVOC level is expressed in parts per billion (ppb) in the range 0-65000.
    pub fn tvoc(&mut self) -> Result<u16, E> {
        self.read_register::<2>(ENS160_DATA_TVOC_REG)
            .map(u16::from_le_bytes)
    }

    /// Returns the Equivalent Carbon Dioxide (eCO2) measurement from the sensor.
    ///
    /// The eCO2 level is expressed in parts per million (ppm) in the range 400-65000.
    pub fn eco2(&mut self) -> Result<ECo2, E> {
        self.read_register::<2>(ENS160_DATA_ECO2_REG)
            .map(u16::from_le_bytes)
            .map(ECo2::from)
    }

    /// Sets the temperature value used in the device's calculations.
    ///
    /// Unit is scaled by 100. For example, a temperature value of 2550 should be used for 25.50 °C.
    pub fn set_temp(&mut self, ambient_temp: i16) -> Result<(), E> {
        let temp = ((ambient_temp as i32 + 27315) * 64 / 100) as u16;
        let temp = temp.to_le_bytes();
        let tbuffer = [ENS160_TEMP_IN_REG, temp[0], temp[1]];
        self.write_register(tbuffer)
    }

    /// Sets the relative humidity value used in the device's calculations.
    ///
    /// Unit is scaled by 100. For example, a humidity value of 5025 should be used for 50.25% RH.
    pub fn set_hum(&mut self, relative_humidity: u16) -> Result<(), E> {
        let rh = (relative_humidity as u32 * 512 / 100) as u16;
        let rh = rh.to_le_bytes();
        let hbuffer = [ENS160_RH_IN_REG, rh[0], rh[1]];
        self.write_register(hbuffer)
    }

    fn read_register<const N: usize>(&mut self, register: u8) -> Result<[u8; N], E> {
        let mut write_buffer = [0u8; 1];
        write_buffer[0] = register;
        let mut buffer = [0u8; N];
        self.i2c
            .write_read(self.address, &write_buffer, &mut buffer)
            .ok();
        Ok(buffer)
    }

    fn write_register<const N: usize>(&mut self, buffer: [u8; N]) -> Result<(), E> {
        self.i2c.write(self.address, &buffer)
    }
}

/// Operation Mode of the sensor.
#[repr(u8)]
enum OperationMode {
    /// DEEP SLEEP mode (low power standby).
    Sleep = 0x00,
    /// IDLE mode (low-power).
    Idle = 0x01,
    /// STANDARD Gas Sensing Modes.
    Standard = 0x02,
    /// Reset device.
    Reset = 0xF0,
}

bitfield! {
    /// Status of the sensor.
    pub struct Status(u8);
    impl Debug;
    pub bool, running_normally, _: 7;
    pub bool, error, _: 6;
    pub into Validity, validity_flag, _: 3,2;
    pub bool, data_is_ready, _: 1;
    pub bool, new_data_in_gpr, _: 0;
}

#[derive(Debug, Clone, Copy)]
pub enum Validity {
    NormalOperation,
    WarmupPhase,
    InitStartupPhase,
    InvalidOutput,
}

impl From<u8> for Validity {
    fn from(v: u8) -> Self {
        match v {
            0b00 => Self::NormalOperation,
            0b01 => Self::WarmupPhase,
            0b10 => Self::InitStartupPhase,
            0b11 => Self::InvalidOutput,
            _ => unreachable!(),
        }
    }
}

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord, Serialize)]
#[repr(u8)]
pub enum AirqualityIndex {
    Excellent = 1,
    Good = 2,
    Moderate = 3,
    Poor = 4,
    Unhealthy = 5,
}

impl From<u8> for AirqualityIndex {
    fn from(i: u8) -> Self {
        match i {
            1 => Self::Excellent,
            2 => Self::Good,
            3 => Self::Moderate,
            4 => Self::Poor,
            5 => Self::Unhealthy,
            _ => Self::Unhealthy,
        }
    }
}

pub struct ECo2(u16);

impl From<u16> for ECo2 {
    fn from(v: u16) -> Self {
        Self(v)
    }
}

impl TryFrom<ECo2> for AirqualityIndex {
    type Error = AirqualityConvError;

    fn try_from(e: ECo2) -> Result<Self, Self::Error> {
        let value = e.0;
        match value {
            400..=599 => Ok(Self::Excellent),
            600..=799 => Ok(Self::Good),
            800..=999 => Ok(Self::Moderate),
            1000..=1499 => Ok(Self::Poor),
            1500..=u16::MAX => Ok(Self::Unhealthy),
            _ => Err(AirqualityConvError(value)),
        }
    }
}

impl Deref for ECo2 {
    type Target = u16;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for ECo2 {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug)]
pub struct AirqualityConvError(pub(crate) u16);

#[cfg(feature = "std")]
impl std::fmt::Display for AirqualityConvError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{} is no valid airquality value. Values start from 400.",
            self.0
        )
    }
}

#[cfg(feature = "std")]
impl std::error::Error for AirqualityConvError {}
