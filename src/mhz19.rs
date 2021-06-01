use embedded_hal::serial::{Write, Read};

#[allow(dead_code)]
#[repr(u8)]
enum Commands {
    ReadConcentration = 0x86,
    CalibrateZeroPoint = 0x87,
    CalibrateSpanPoint = 0x88,
    AutoCalibration = 0x79,
    SetRange = 0x99
}

pub enum Range {
    _1000,
    _2000,
    _3000,
    _5000,
    _10000
}

pub enum Errors {
    Write,
    Read,
    Checksum
}

pub enum AutoCalibrationState {
    Enable,
    Disable
}

// Calculate checksum for mhz19 packet
fn checksum(data: &[u8]) -> u8 {
    let mut result: u8 = 0;

    for &number in data {
        match result.overflowing_add(number) {
            (value, _) => { result = value; }
        }
    }

    match 255_u8.overflowing_sub(result) {
        (value, _) => { result = value; }
    }

    result
}

/// Trait for implement mhz-19 logic
pub trait Mhz19Trait {
    type Error;

    fn co2(&mut self) -> Result<u16, Self::Error>;
    fn auto_calibration(&mut self, state: AutoCalibrationState) -> Result<(), Self::Error>;
    fn range(&mut self, range: Range) -> Result<(), Self::Error>;
}

const BUFFER_SIZE: usize = 9;

///
/// Mhz-19 implementation of the driver for transmission for serial.
/// Need set serial baudrate 9600
///
/// # Example
///
/// ```
/// let serial = Serial::new(...);
/// let mut mhz = Mhz19::new(serial);
///
/// let co2: u16 = mhz.co2().unwrap();
/// ```
pub struct Mhz19<SerialType>
    where
        SerialType: Read<u8> + Write<u8>
{
    serial: SerialType,
    buffer: [u8; BUFFER_SIZE]
}


impl<SerialType> Mhz19<SerialType>
    where
        SerialType: Read<u8> + Write<u8>
{
    pub fn new(serial: SerialType) -> Self {
        Self {
            serial,
            buffer: [0; BUFFER_SIZE]
        }
    }

    /// Send command to mhz-19 over serial
    fn command(&mut self, cmd: u8, data: [u8; 5]) -> Result<(), Errors> {
        self.buffer = [
            0xFF,
            0x01,
            cmd,
            data[0],
            data[1],
            data[2],
            data[3],
            data[4],
            0x00
        ];

        let crc_index = BUFFER_SIZE - 1;

        self.buffer[crc_index] = checksum(&self.buffer[0..crc_index]);

        for &b in self.buffer.iter() {
            match nb::block!(self.serial.write(b)) {
                Err(_) => { return Err(Errors::Write); }
                _ => {}
            }
        }

        Ok(())
    }

    /// Read response from mhz-19 driver with check checksum
    fn response(&mut self) -> Result<(), Errors> {
        for index in 0..BUFFER_SIZE {
            match nb::block!(self.serial.read()) {
                Ok(data) => { self.buffer[index] = data; }
                Err(_) => { return Err(Errors::Read); }
            }
        }

        let crc_index = BUFFER_SIZE - 1;

        if checksum(&self.buffer[0..crc_index]) != self.buffer[crc_index] {
            return Err(Errors::Checksum);
        }

        Ok(())
    }
}

impl<SerialType> Mhz19Trait for Mhz19<SerialType>
    where
        SerialType: Read<u8> + Write<u8>
{
    type Error = Errors;

    /// Get gas concentration from mhz-19
    fn co2(&mut self) -> Result<u16, Self::Error> {
        let data: [u8; 5] = [0; 5];

        self.command(Commands::ReadConcentration as u8, data)?;
        self.response()?;

        let result: u16 = ((self.buffer[2] as u16) << 8_u16) | (self.buffer[3] as u16);

        Ok(result)
    }

    ///
    /// Set auto calibration or not for mhz-19 driver
    ///
    /// # Example
    ///
    /// ```
    /// let serial = Serial::new(...);
    /// let mut mhz = Mhz19::new(serial);
    ///
    /// mhz.auto_calibration(AutoCalibrationState::Disable).unwrap();
    /// ```
    ///
    fn auto_calibration(&mut self, state: AutoCalibrationState) -> Result<(), Self::Error> {
        let state_byte: u8 = match state {
            AutoCalibrationState::Enable => { 0xA0 }
            AutoCalibrationState::Disable => { 0x00 }
        };

        let data: [u8; 5] = [state_byte, 0, 0, 0, 0];

        self.command(Commands::AutoCalibration as u8, data)
    }

    ///
    /// Set maximum range for mhz-19 conversation (from 0 to range value)
    ///
    /// # Example
    ///
    /// ```
    /// let serial = Serial::new(...);
    /// let mut mhz = Mhz19::new(serial);
    ///
    /// mhz.range(Range::_2000).unwrap();
    /// ```
    fn range(&mut self, range: Range) -> Result<(), Self::Error> {
        let data: [u8; 5] = match range {
            Range::_1000 => { [0x00, 0x00, 0x00, 0x03, 0xE8] }
            Range::_2000 => { [0x00, 0x00, 0x00, 0x07, 0xD0] }
            Range::_3000 => { [0x00, 0x00, 0x00, 0x0B, 0xB8] }
            Range::_5000 => { [0x00, 0x00, 0x00, 0x13, 0x88] }
            Range::_10000 => { [0x00, 0x00, 0x00, 0x27, 0x10] }
        };

        self.command(Commands::SetRange as u8, data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use heapless::{Vec, consts};

    struct DummySerial<'a> {
        input: &'a mut Vec<u8, consts::U9>,
        output: &'a mut Vec<u8, consts::U9>
    }

    impl<'a> DummySerial<'a> {
        fn new(
            input: &'a mut Vec<u8, consts::U9>,
            output: &'a mut Vec<u8, consts::U9>
        ) -> Self
        {
            Self {
                input,
                output
            }
        }
    }

    impl<'a> Write<u8> for DummySerial<'a> {
        type Error = nb::Error<()>;

        fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
            self.input.push(word).unwrap();

            Ok(())
        }

        fn flush(&mut self) -> nb::Result<(), Self::Error> { Ok(()) }
    }

    impl<'a> Read<u8> for DummySerial<'a> {
        type Error = nb::Error<()>;

        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            return match self.output.pop() {
                Some(data) => { Ok(data) }
                _ => { Err(nb::Error::WouldBlock) }
            };
        }
    }

    #[test]
    fn test_checksum() {
        // example data from datasheet
        let test_data_1: [u8; 9] = [0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79];
        let test_data_2: [u8; 9] = [0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78];
        let test_data_3: [u8; 9] = [0xFF, 0x01, 0x88, 0x07, 0xD0, 0x00, 0x00, 0x00, 0xA0];

        assert_eq!(checksum(&test_data_1[0..8]), test_data_1[8]);
        assert_eq!(checksum(&test_data_2[0..8]), test_data_2[8]);
        assert_eq!(checksum(&test_data_3[0..8]), test_data_3[8]);
    }

    #[test]
    fn command_test() {
        let mut input: Vec<u8, consts::U9> = Vec::new();
        let mut _output: Vec<u8, consts::U9> = Vec::new();

        let serial = DummySerial::new(&mut input, &mut _output);

        let mut mhz = Mhz19::new(serial);

        match mhz.command(0x86, [0_u8; 5]) {
            Ok(_) => {}
            _ => { panic!("Error write command"); }
        }

        let expected_answer: [u8; 9] = [
            0xFF,
            0x01,
            0x86,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x79
        ];

        assert_eq!(input, expected_answer);
    }

    #[test]
    fn response_test() -> Result<(), &'static str> {
        let mut _input: Vec<u8, consts::U9> = Vec::new();
        let mut output: Vec<u8, consts::U9> = Vec::new();

        let mut packet: [u8; 9] = [
            0xFF,
            0x01,
            0x04,
            0x80,
            0xB0,
            0x00,
            0x00,
            0x00,
            0x00,
        ];

        packet[8] = checksum(&packet[0..8]);

        for &b in packet.iter() {
            output.push(b).unwrap();
        }

        let serial = DummySerial::new(&mut _input, &mut output);

        let mut mhz = Mhz19::new(serial);

        return match mhz.response() {
            Ok(_) => { Ok(()) }
            _ => { Err("Can't read successful response") }
        };
    }
}
