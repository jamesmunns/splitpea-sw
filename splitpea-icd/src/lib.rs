#![no_std]

use serde::{Serialize, Deserialize};

#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Debug, Serialize, Deserialize)]
pub struct Event {
    // TODO(AJM): Should these be pub?
    pub kidx: u8,
    pub kind: EventKind,
}

#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
#[derive(Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum EventKind {
    KeyPress,
    KeyRelease,
    // TODO: KeyHold?
}
