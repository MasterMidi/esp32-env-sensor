use embedded_hal::blocking::i2c::*;

pub trait I2CSensor<T, E>
where
    T: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
    /// Creates a new sensor driver.
    fn new(i2c: T, address: u8) -> Self;

    /// Releases the underlying I2C bus and destroys the driver.
    fn release(self) -> T;
}
