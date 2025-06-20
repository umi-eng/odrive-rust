#![cfg_attr(not(test), no_std)]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

/// 11-bit identifier mask.
const ID_MASK: u16 = 0x7FF;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Id(u16);

impl Id {
    /// Creates a new cansimple identifier.
    ///
    /// Will return [`None`] if `node` is > 63 or `command` is > 31.
    pub fn new(node: u8, command: u8) -> Option<Self> {
        if node <= 0b111111 && command <= 0b11111 {
            let node = node as u16;
            let command = command as u16;

            Some(Self((node << 5) | command))
        } else {
            None
        }
    }

    /// Create a new ['Id'] from a raw identifier value.
    ///
    /// Masked to 11 bits to ensure the id is valid.
    pub fn from_raw(raw: u16) -> Self {
        Self(raw & ID_MASK)
    }

    /// Get the raw identifier value.
    pub fn as_raw(&self) -> u16 {
        self.0
    }

    /// Command identifier.
    pub fn command(&self) -> u8 {
        (self.0 & 0x1F) as u8
    }

    /// Node identifier.
    pub fn node(&self) -> u8 {
        (self.0 >> 5) as u8
    }
}

impl From<embedded_can::StandardId> for Id {
    fn from(value: embedded_can::StandardId) -> Self {
        Id(value.as_raw())
    }
}

impl From<Id> for embedded_can::StandardId {
    fn from(id: Id) -> Self {
        embedded_can::StandardId::new(id.as_raw()).unwrap()
    }
}

impl From<Id> for embedded_can::Id {
    fn from(id: Id) -> Self {
        embedded_can::Id::Standard(id.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identifier_parse() {
        // example: odrive node: 1, cmd: get encoder estimates
        let id = Id::from_raw(0x029);
        assert_eq!(id.node(), 1);
        assert_eq!(id.command(), 9);
    }

    #[test]
    fn make_identifier() {
        let id = Id::new(1, 9).unwrap();
        assert_eq!(id.as_raw(), 0x029);
    }
}
