#![no_main]
#![no_std]

use heapless::spsc::Producer;
use keyberon::action::Action;
use stem_bringup as _; // global logger + panicking-behavior + memory layout
use nrf52840_hal::{
    clocks::{ExternalOscillator, Internal, LfOscStopped},
    gpio::{p0::Parts as P0Parts, p1::Parts as P1Parts, Level},
    pac::{SPIM2, TIMER0, TWIM0},
    spim::{Frequency as SpimFreq, Pins as SpimPins, Spim, MODE_0},
    twim::{Frequency as TwimFreq, Pins as TwimPins, Twim},
    uarte::{Baudrate, Parity, Pins as UartPins, Uarte},
    usbd::{UsbPeripheral, Usbd},
    Clocks,
};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid}, class::UsbClass,
};

// Keyberon
use keyberon::{
    action::{
        k,
        l,
        Action::*,
    },
    hid::HidClass,
    // impl_heterogenous_array,
    key_code::{KbHidReport, KeyCode, KeyCode::*},
    layout::{CustomEvent, Event, Layout},
    matrix::{Matrix, PressedKeys},
};

type UsbClassKey = keyberon::Class<'static, Usbd<UsbPeripheral<'static>>, Leds>;
type UsbDeviceKey = usb_device::device::UsbDevice<'static, Usbd<UsbPeripheral<'static>>>;

pub struct Leds {
    //     caps_lock: gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>,
}

// Constant to denote something I should fix (vs Trans, which is just a nothing)
const TODO: Action = Action::Trans;

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers = &[
    // Left Keyboard, Base Layer
    &[
        &[k(Q),      k(W),     k(E),    k(R),    k(T),     ],
        &[k(A),      k(S),     k(D),    k(F),    k(G),     ],
        &[k(LShift), k(Z),     k(X),    k(C),    k(V),     ],
        &[k(LCtrl),  TODO,     Trans,   k(LGui), k(Space), ],
        //                     ^^^^^ - Not a real key!
    ],
    // &[
    //     &[k(Grave),    k(F1),      k(F2),    k(F3),     k(F4),       k(F5),      k(F6),      k(F7)      ],
    //     &[k(F8),       k(F9),      k(F10),   k(F11),    k(F12),      k(Delete),  Trans,      Trans      ],
    //     &[k(PgDown),   k(Up),      k(PgUp),  Trans,     Trans,       Trans,      Trans,      Trans      ],
    //     &[Trans,       Trans,      Trans,    Trans,     Trans,       Trans,      Trans,      Trans      ],
    //     &[Trans,       Trans,      Trans,    Trans,     Trans,       k(Left),    k(Down),    k(Right)   ],
    //     &[Trans,       Trans,      Trans,    Trans,     Trans,       Trans,      Trans,      Trans      ],
    //     &[Trans,       Trans,      Trans,    Trans,     Trans,       Trans,      Trans,      Trans      ],
    //     &[Trans,       l(1),       Trans,    Trans,     Trans,       Trans,      Trans,      Trans      ],
    // ],
];

impl keyberon::keyboard::Leds for Leds {
    /// Sets the num lock state.
    fn num_lock(&mut self, status: bool) {
        defmt::println!("num_lock: {:?}", status)
    }
    /// Sets the caps lock state.
    fn caps_lock(&mut self, status: bool) {
        defmt::println!("caps_lock: {:?}", status)
    }
    /// Sets the scroll lock state.
    fn scroll_lock(&mut self, status: bool) {
        defmt::println!("scroll_lock: {:?}", status)
    }
    /// Sets the compose state.
    fn compose(&mut self, status: bool) {
        defmt::println!("compose: {:?}", status)
    }
    /// Sets the kana state.
    fn kana(&mut self, status: bool) {
        defmt::println!("kana: {:?}", status)
    }
}

#[rtic::app(device = nrf52840_hal::pac, dispatchers = [SWI0_EGU0])]
mod app {
    use super::*;
    use cortex_m::singleton;
    use defmt::unwrap;
    use heapless::spsc::{Queue, Producer, Consumer};
    use stem_bringup::monotonic::{ExtU32, MonoTimer};
    use splitpea_driver::{Splitpea, icd::EventKind as SpEventKind};

    #[monotonic(binds = TIMER0, default = true)]
    type Monotonic = MonoTimer<TIMER0>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        // interface_comms: InterfaceComms<8>,
        // worker: Worker<WorkerComms<8>, Twim<TWIM0>, Spim<SPIM2>, PhmUart>,
        usb_class: HidClass<'static, Usbd<UsbPeripheral<'static>>, keyberon::keyboard::Keyboard<Leds>>,
        usb_dev: UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
        pea: Splitpea<Twim<TWIM0>>,
        kbd_prod: Producer<'static, KbHidReport, 8>,
        kbd_cons: Consumer<'static, KbHidReport, 8>,
        layout: Layout,
    }

    #[init(local = [
        usb_bus: Option<UsbBusAllocator<Usbd<UsbPeripheral<'static>>>> = None,
        kbd_evts: Queue<KbHidReport, 8> = Queue::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let device = cx.device;

        // Setup clocks early in the process. We need this for USB later
        let clocks = Clocks::new(device.CLOCK);
        let clocks = clocks.enable_ext_hfosc();
        let clocks =
            unwrap!(singleton!(: Clocks<ExternalOscillator, Internal, LfOscStopped> = clocks));

        // Configure the monotonic timer, currently using TIMER0, a 32-bit, 1MHz timer
        let mono = Monotonic::new(device.TIMER0);

        // Create GPIO ports for pin-mapping
        let port0 = P0Parts::new(device.P0);
        let port1 = P1Parts::new(device.P1);

        // Set up Twim
        let i2c = Twim::new(
            device.TWIM0,
            TwimPins {
                scl: port1.p1_01.into_floating_input().degrade(),
                sda: port1.p1_02.into_floating_input().degrade(),
            },
            TwimFreq::K100,
        );
        let pea = Splitpea::new(i2c);

        // Set up USB Serial Port
        let usb_bus = cx.local.usb_bus;
        usb_bus.replace(Usbd::new(UsbPeripheral::new(device.USBD, clocks)));

        let leds = Leds { };

        let usb_class = keyberon::new_class(usb_bus.as_ref().unwrap(), leds);
        let usb_dev = keyberon::new_device(usb_bus.as_ref().unwrap());

        let (kbd_prod, kbd_cons) = cx.local.kbd_evts.split();

        usb_tick::spawn().ok();
        key_tick::spawn().ok();
        (
            Shared {},
            Local {
                kbd_prod,
                kbd_cons,
                usb_class,
                usb_dev,
                pea,
                layout: Layout::new(LAYERS),
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [pea, kbd_prod, layout, ctr: u32 = 0])]
    fn key_tick(cx: key_tick::Context) {
        let pea = cx.local.pea;
        let layout = cx.local.layout;

        *cx.local.ctr += 1;
        if *cx.local.ctr >= 100 {
            *cx.local.ctr = 0;
            defmt::println!("KEY TICK");
        }

        let evts = defmt::unwrap!(pea.get_all_events().map_err(drop));

        for evt in evts {
            defmt::println!("EVENT: {:?}", evt);

            let idx_x = evt.kidx / 5;
            let idx_y = evt.kidx % 5;

            let kevt = match evt.kind {
                SpEventKind::KeyPress => {
                    Event::Press(idx_x, idx_y)
                },
                SpEventKind::KeyRelease => {
                    Event::Release(idx_x, idx_y)
                },
            };

            layout.event(kevt);
        }

        match layout.tick() {
            CustomEvent::Press(p) => defmt::println!("press {:?}", p),
            CustomEvent::Release(r) => defmt::println!("release {:?}", r),
            _ => {}
        }

        let rpt: KbHidReport = layout.keycodes().collect();
        cx.local.kbd_prod.enqueue(rpt).ok();

        key_tick::spawn_after(10.millis()).ok();
    }

    #[task(local = [usb_dev, usb_class, kbd_cons, ctr: u32 = 0, hold: Option<(KbHidReport, usize)> = None])]
    fn usb_tick(cx: usb_tick::Context) {
        let usb_dev = cx.local.usb_dev;
        let usb_class = cx.local.usb_class;
        // let cobs_buf = cx.local.cobs_buf;
        // let interface_comms = cx.local.interface_comms;

        *cx.local.ctr += 1;
        if *cx.local.ctr >= 1000 {
            *cx.local.ctr = 0;
            defmt::println!("USB TICK");
        }

        let mut buf = [0u8; 128];

        usb_poll(usb_dev, usb_class);

        let rpt = cx.local.hold.take();
        let rpt = rpt.or_else(|| {
            // Only attempt to send a report if we HAVE one, AND if
            // it is "new information"
            if let Some(r) = cx.local.kbd_cons.dequeue() {
                if usb_class.device_mut().set_keyboard_report(r.clone()) {
                    Some((r, 0))
                } else {
                    None
                }
            } else {
                None
            }
        });

        // Do we have some kind of pending report to send?
        if let Some((rpt, offset)) = rpt {
            let rptb = &rpt.as_bytes()[offset..];
            match usb_class.write(rptb) {
                Ok(0) => {
                    // We do nothing, save off the remaining report for next
                    // try
                    *cx.local.hold = Some((rpt, offset));
                }
                Ok(n) => {
                    if rptb.len() == n {
                        // We wrote everything, yay!
                    } else {
                        // We wrote *some* of the message, save it (and the offset) off to retry later
                        *cx.local.hold = Some((rpt, offset + n));
                    }
                }
                Err(_e) => {
                    defmt::panic!();
                }
            }
        }

        usb_tick::spawn_after(1.millis()).ok();
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::println!("Hello, world!");
        // let worker = cx.local.worker;

        loop {
            // unwrap!(worker.step().map_err(drop));
            cortex_m::asm::nop();
        }
    }
}

fn usb_poll(usb_dev: &mut UsbDeviceKey, keyboard: &mut UsbClassKey) {
    if usb_dev.poll(&mut [keyboard]) {
        keyboard.poll();
    }
}
