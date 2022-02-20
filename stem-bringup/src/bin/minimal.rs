#![no_main]
#![no_std]

use stem_bringup as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = nrf52840_hal::pac)]
mod app {
    // TODO: Add a monotonic if scheduling will be used
    // #[monotonic(binds = SysTick, default = true)]
    // type DwtMono = DwtSystick<80_000_000>;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::println!("init");

        // task1::spawn().ok();

        // Setup the monotonic timer
        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
            },
            init::Monotonics(
                // Initialization of optional monotonic timers go here
            ),
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::println!("idle");

        loop {
            continue;
        }
    }

    // // TODO: Add tasks
    // #[task]
    // fn task1(_cx: task1::Context) {
    //     defmt::println!("Hello from task1!");
    // }
}
