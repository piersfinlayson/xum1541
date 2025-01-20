#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]

pub enum Protocol {
    Cbm = (1 << 4),           // Standard CBM protocol
    S1 = (2 << 4),            // serial1
    S2 = (3 << 4),            // serial2
    PP = (4 << 4),            // parallel1
    P2 = (5 << 4),            // 2 byte parallel
    Nib = (6 << 4),           // burst nibbler parallel
    NibCommand = (7 << 4),    // BN parallel commands
    NibSrq = (8 << 4),        // 1571 Serial nibbler
    NibSrqCommand = (9 << 4), // Serial commands
}
pub enum Write {
    Talk = (1 << 0),
    Atn = (1 << 1),
}
