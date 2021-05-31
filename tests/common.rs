use embedded_hal::serial::{Read, Write};
use std::collections::VecDeque;

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

pub struct DummySerial {
    input: VecDeque<u8>,
    output: VecDeque<u8>
}

impl DummySerial {
    pub fn new() -> Self {
        Self {
            input: VecDeque::new(),
            output: VecDeque::new()
        }
    }

    fn process(&mut self, data: [u8; 9]) {
        if data[0] != 0xFF { return; }
        if data[1] != 0x01 { return; }

        let packet = match data[2] {
            0x86 => {
                let mut packet: [u8; 9] = [
                    0xFF,
                    0x01,
                    0x04,
                    0xB0,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                ];

                packet[8] = checksum(&packet[0..8]);
                packet
            }
            0x87 => { return; } // For this command no return value
            0x88 => { return; } // For this command no return value
            _ => { panic!("unexpected command!"); }
        };

        for &b in packet.iter() {
            self.output.push_back(b);
        }
    }

    fn receive(&mut self, data: u8) {
        self.input.push_back(data);

        if self.input.len() == 9 {
            let mut parse_buffer: [u8; 9] = [0; 9];

            for d in parse_buffer.iter_mut() {
                match self.input.pop_front() {
                    Some(inp) => { *d = inp; }
                    _ => {}
                }
            }

            if checksum(&parse_buffer[0..8]) == parse_buffer[8] {
                self.process(parse_buffer);
            }
        }
    }
}

impl Write<u8> for DummySerial {
    type Error = nb::Error<()>;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.receive(word);

        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> { Ok(()) }
}

impl Read<u8> for DummySerial {
    type Error = nb::Error<()>;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if self.output.is_empty() {
            return Err(nb::Error::WouldBlock);
        }

        if let Some(data) = self.output.pop_front() {
            return Ok(data);
        };

        Err(nb::Error::WouldBlock)
    }
}
