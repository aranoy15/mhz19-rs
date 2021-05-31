
mod common;

#[cfg(test)]
mod tests {
    use mhz19_rs::mhz19::{Mhz19, Mhz19Trait};
    use crate::common;

    #[test]
    fn co2_test() {
        let serial = common::DummySerial::new();
        let mut mhz = Mhz19::new(serial);

        let co2: u16 = mhz.co2().unwrap_or(0_u16);

        assert_eq!(co2, 1200_u16);
    }
}
