#[derive(Debug, Clone)]
pub enum Command {
    Init,
    Reset,
    Read,
    Write,
    Talk { device: u8, secondary: u8 },
    Listen { device: u8, secondary: u8 },
    Untalk,
    Unlisten,
    Open { device: u8, secondary: u8 },
    Close { device: u8, secondary: u8 },
    ClockSet,
    Wait,
    Shutdown,
}

pub trait IecCommand {
    fn command(&self) -> Command;
    fn device(&self) -> Option<u8> {
        None
    }
    fn secondary(&self) -> Option<u8> {
        None
    }
}

// Standard IEC commands implementation
impl IecCommand for Command {
    fn command(&self) -> Command {
        self.clone()
    }

    fn device(&self) -> Option<u8> {
        match self {
            Command::Talk { device, .. }
            | Command::Listen { device, .. }
            | Command::Open { device, .. }
            | Command::Close { device, .. } => Some(*device),
            _ => None,
        }
    }

    fn secondary(&self) -> Option<u8> {
        match self {
            Command::Talk { secondary, .. }
            | Command::Listen { secondary, .. }
            | Command::Open { secondary, .. }
            | Command::Close { secondary, .. } => Some(*secondary),
            _ => None,
        }
    }
}
