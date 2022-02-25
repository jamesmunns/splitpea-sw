#![cfg_attr(not(feature = "use-std"), no_std)]

pub use splitpea_icd as icd;
use embedded_hal::blocking::i2c::WriteRead;
use core::fmt::Debug;

#[derive(Debug, PartialEq)]
pub enum Error<T: Debug> {
    I2c(T),
    Postcard(postcard::Error),
}

impl<T: Debug> From<postcard::Error> for Error<T> {
    fn from(pe: postcard::Error) -> Self {
        Error::Postcard(pe)
    }
}

pub struct Splitpea<I2C>
where
    I2C: WriteRead,
{
    i2c: I2C,
    addr: u8,
}

impl<I2C> Splitpea<I2C>
where
    I2C: WriteRead,
    I2C::Error: Debug,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            addr: 0x42,
        }
    }

    pub fn new_with_addr(i2c: I2C, addr: u8) -> Self {
        Self {
            i2c,
            addr,
        }
    }

    pub fn get_event_cts(&mut self) -> Result<usize, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[0x10], &mut buf).map_err(Error::I2c)?;
        Ok(buf[0] as usize)
    }

    // TODO: Magic number
    pub fn get_all_events(&mut self) -> Result<heapless::Vec<icd::Event, 32>, Error<I2C::Error>> {
        let ct = self.get_event_cts()?;

        if ct == 0 {
            return Ok(heapless::Vec::new());
        }

        let mut resp = heapless::Vec::new();
        let mut buf = [0u8; 64]; // TODO: Magic number
        let bufsl = &mut buf [..(2 * ct)];
        self.i2c.write_read(self.addr, &[0x20, ct as u8], bufsl).map_err(Error::I2c)?;

        for ch in bufsl.chunks_exact(2) {
            let event: icd::Event = postcard::from_bytes(ch)?;
            resp.push(event).ok();
        }

        Ok(resp)
    }
}

#[cfg(feature = "use-std")]
impl<I2C> Splitpea<I2C>
where
    I2C: WriteRead,
    I2C::Error: Debug,
{
    pub fn get_all_events_vec(&mut self) -> Result<Vec<icd::Event>, Error<I2C::Error>> {
        let ct = self.get_event_cts()?;

        if ct == 0 {
            return Ok(vec![]);
        }

        let mut resp = Vec::new();
        let mut buf = [0u8; 64]; // TODO: Magic number
        let bufsl = &mut buf [..(2 * ct)];
        self.i2c.write_read(self.addr, &[0x20, ct as u8], bufsl).map_err(Error::I2c)?;

        for ch in bufsl.chunks_exact(2) {
            let event: icd::Event = postcard::from_bytes(ch)?;
            resp.push(event);
        }

        Ok(resp)
    }
}
