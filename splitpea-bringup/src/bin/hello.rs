#![no_main]
#![no_std]

use splitpea_bringup as _; // global logger + panicking-behavior + memory layout

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");

    splitpea_bringup::exit()
}
