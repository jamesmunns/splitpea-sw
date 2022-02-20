#![no_std]
#![cfg_attr(test, no_main)]

use stem_bringup as _; // memory layout + panic handler

#[defmt_test::tests]
mod tests {}
