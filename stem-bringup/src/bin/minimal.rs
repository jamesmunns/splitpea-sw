#![no_main]
#![no_std]

use cortex_m::peripheral::SCB;
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

use groundhog::RollingTimer;
use groundhog_nrf52::GlobalRollingTimer;

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

/// Keyboard layout
///
/// [Reference Drawing](https://twitter.com/bitshiftmask/status/1483065694744940545)
#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers = &[
    //
    // Base Layer
    //
    // Left Keyboard                                           Right Keyboard
    &[
        &[k(Q),      k(W), k(E),  k(R),     k(T), /*  */ k(Y), k(U), k(I),  k(O),     k(P),      ],
        &[k(A),      k(S), k(D),  k(F),     k(G), /*  */ k(H), k(J), k(K),  k(L),     k(Enter),  ],
        &[k(LShift), k(Z), k(X),  k(C),     k(V), /*  */ k(B), k(N), k(M),  k(Comma), k(Dot),    ],
        &[k(LCtrl),  TODO, Trans, k(Space), l(2), /*  */ l(1), TODO, Trans, k(LGui),  k(BSpace), ],
        //                 ^^^^^ - Not a real key!                   ^^^^^ - Not a real key!
    ],

    //
    // Num Layer
    //
    // Left Keyboard                                        Right Keyboard
    &[
        &[k(Kb1),    k(Kb2), k(Kb3), k(Kb4), k(Kb5), /*  */ k(Kb6),  k(Kb7),  k(Kb8),    k(Kb9),      k(Kb0),      ],
        &[TODO,      TODO,   TODO,   TODO,   TODO,   /*  */ k(PgUp), k(Up),   k(PgDown), k(LBracket), k(RBracket), ],
        &[k(LShift), TODO,   TODO,   TODO,   TODO,   /*  */ k(Left), k(Down), k(Right),  k(Minus),    k(Equal),    ],
        &[k(LCtrl),  TODO,   Trans,  TODO,   l(2),   /*  */ l(1),    TODO,    Trans,     k(LGui),     TODO,        ],
        //                   ^^^^^ - Not a real key!                          ^^^^^ - Not a real key!
    ],

    //
    // Fancy Layer
    //
    // Left Keyboard                                   Right Keyboard
    &[
        &[k(Escape), TODO,   TODO,  TODO, TODO, /*  */ TODO, TODO, TODO,  TODO,    k(BSpace), ],
        &[k(Grave),  k(Tab), TODO,  TODO, TODO, /*  */ TODO, TODO, TODO,  TODO,    TODO,      ],
        &[k(LShift), TODO,   TODO,  TODO, TODO, /*  */ TODO, TODO, TODO,  TODO,    TODO,      ],
        &[k(LCtrl),  TODO,   Trans, TODO, l(2), /*  */ l(1), TODO, Trans, k(LGui), TODO,      ],
        //                   ^^^^^ - Not a real key!               ^^^^^ - Not a real key!
    ],

    // //
    // // XXX Layer
    // //
    // // Left Keyboard                              Right Keyboard
    // &[
    //     &[TODO, TODO, TODO,  TODO, TODO, /*  */ TODO, TODO, TODO,  TODO, TODO, ],
    //     &[TODO, TODO, TODO,  TODO, TODO, /*  */ TODO, TODO, TODO,  TODO, TODO, ],
    //     &[TODO, TODO, TODO,  TODO, TODO, /*  */ TODO, TODO, TODO,  TODO, TODO, ],
    //     &[TODO, TODO, Trans, TODO, TODO, /*  */ TODO, TODO, Trans, TODO, TODO, ],
    //     //            ^^^^^ - Not a real key!               ^^^^^ - Not a real key!
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
    use cortex_m::{singleton, prelude::_embedded_hal_adc_OneShot};
    use defmt::unwrap;
    use heapless::spsc::{Queue, Producer, Consumer};
    use nrf52840_hal::{gpio::{Pin, Output, PushPull, p0::P0_02, Input, Floating}, prelude::{OutputPin, InputPin}, Saadc, saadc::{SaadcConfig, Resolution, Reference, Gain, Resistor, Oversample, Time}};
    use stem_bringup::monotonic::{ExtU32, MonoTimer};
    use splitpea_driver::{Splitpea, icd::EventKind as SpEventKind};
    use shared_bus::{BusManager, NullMutex};

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
        bus: BusManager<NullMutex<Twim<TWIM0>>>,
        kbd_prod: Producer<'static, KbHidReport, 8>,
        kbd_cons: Consumer<'static, KbHidReport, 8>,
        led_1: Pin<Output<PushPull>>,
        led_2: Pin<Output<PushPull>>,
        saadc: Saadc,
        adc_pin: P0_02<Input<Floating>>,
        stat_pin: Pin<Input<Floating>>,

        layout: Layout,
    }

    #[init(local = [
        usb_bus: Option<UsbBusAllocator<Usbd<UsbPeripheral<'static>>>> = None,
        kbd_evts: Queue<KbHidReport, 8> = Queue::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let device = cx.device;

        // defmt::println!("Hello!");
        // stem_bringup::exit();

        disable_nfc_pins(&device);

        // Setup clocks early in the process. We need this for USB later
        let clocks = Clocks::new(device.CLOCK);
        let clocks = clocks.enable_ext_hfosc();
        let clocks =
            unwrap!(singleton!(: Clocks<ExternalOscillator, Internal, LfOscStopped> = clocks));

        // Configure the monotonic timer, currently using TIMER0, a 32-bit, 1MHz timer
        let mono = Monotonic::new(device.TIMER0);
        GlobalRollingTimer::init(device.TIMER1);

        // Create GPIO ports for pin-mapping
        let port0 = P0Parts::new(device.P0);
        let port1 = P1Parts::new(device.P1);

        // Enable 3v3 reg
        let _ = port1.p1_13.into_push_pull_output(Level::High);

        let led_1 = port0.p0_09.into_push_pull_output(Level::High).degrade();
        let led_2 = port0.p0_10.into_push_pull_output(Level::High).degrade();

        // Set up Twim
        let i2c = Twim::new(
            device.TWIM0,
            TwimPins {
                scl: port0.p0_08.into_floating_input().degrade(),
                sda: port0.p0_06.into_floating_input().degrade(),
            },
            TwimFreq::K100,
        );
        let bus = shared_bus::BusManagerSimple::new(i2c);

        // Set up SAADC engine. At the moment, this is only used for current
        // sense, though it might need to be more general in the future to
        // support ADC pins on the EXT interface.
        let saadc = Saadc::new(
            device.SAADC,
            SaadcConfig {
                // NOTE: Proooooobably overkill, but whatever.
                resolution: Resolution::_14BIT,

                // NOTE: Total range is (ref / gain), so a reference of 1/4 and
                // a gain of 1/4 gives us 0..VDD (3.3v) range.
                reference: Reference::VDD1_4,
                gain: Gain::GAIN1_4,

                resistor: Resistor::BYPASS,

                oversample: Oversample::OVER64X,
                time: Time::_40US,
            },
        );

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
                bus,
                led_1,
                led_2,
                saadc,
                adc_pin: port0.p0_02.into_floating_input(),
                stat_pin: port1.p1_09.into_floating_input().degrade(),
                layout: Layout::new(LAYERS),
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [bus, kbd_prod, layout, ctr: u32 = 0])]
    fn key_tick(cx: key_tick::Context) {
        let bus = cx.local.bus;
        let layout = cx.local.layout;

        *cx.local.ctr += 1;
        if *cx.local.ctr >= 100 {
            *cx.local.ctr = 0;
            // defmt::println!("KEY TICK");
        }

        let mut pea1 = Splitpea::new_with_addr(bus.acquire_i2c(), 0x42);
        let mut pea2 = Splitpea::new_with_addr(bus.acquire_i2c(), 0x44);

        //////////////////////
        // LEFT SIDE
        //////////////////////
        let evts = defmt::unwrap!(pea1.get_all_events().map_err(drop));
        for evt in evts {
            defmt::println!("EVENT: {:?}", evt);

            let idx_x = evt.kidx / 5;
            let idx_y = evt.kidx % 5;

            defmt::println!("x:{=u8}, y:{=u8}", idx_x, idx_y);

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

        //////////////////////
        // RIGHT SIDE
        //////////////////////
        let evts = defmt::unwrap!(pea2.get_all_events().map_err(drop));
        for evt in evts {
            defmt::println!("EVENT: {:?}", evt);

            let idx_x = evt.kidx / 5;
            let idx_y = 5 + (evt.kidx % 5);

            defmt::println!("x:{=u8}, y:{=u8}", idx_x, idx_y);

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
            // defmt::println!("USB TICK");
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

    #[idle(local = [led_1, led_2, saadc, adc_pin, stat_pin])]
    fn idle(cx: idle::Context) -> ! {
        defmt::println!("Hello, world!");
        // let worker = cx.local.worker;

        let led_1 = cx.local.led_1;
        let led_2 = cx.local.led_2;
        let saadc = cx.local.saadc;
        let adc_pin = cx.local.adc_pin;
        let stat_pin = cx.local.stat_pin;

        let timer = GlobalRollingTimer::default();
        let mut start = timer.get_ticks();
        let mut toggle = false;

        const RDIV_TOP: f32 = 47_000.0;
        const RDIV_BOT: f32 = 68_000.0;
        const RDIV_RATIO: f32 = RDIV_BOT / (RDIV_TOP + RDIV_BOT);
        const ADC_RANGE_MAX: f32 = 0x3FFF as f32;
        const ADC_VOLT_MAX: f32 = 3.3;

        loop {
            if timer.millis_since(start) >= 500 {
                if toggle {
                    let reading = defmt::unwrap!(saadc.read(adc_pin).map_err(drop));

                    //           (reading / range_max) * adc_voltage       perceived voltage
                    // voltage = ----------------------------------- = ------------------------
                    //            rdiv_top / (rdiv_top + rdiv_bot)      resistor divider ratio
                    let voltage = (((reading as f32) / ADC_RANGE_MAX) * ADC_VOLT_MAX) / RDIV_RATIO;

                    // TODO: Map meanings
                    //
                    // WHEN USB Connected + CHARGING:
                    // When batt disconnected: true
                    // When batt connected: false
                    //
                    // When USB !Connected + !CHARGING:
                    // always false?
                    //

                    let stat = defmt::unwrap!(stat_pin.is_high().map_err(drop));
                    defmt::println!("voltage: {=f32}, stat: {=bool}", voltage, stat);

                    led_1.set_low().ok();
                    led_2.set_high().ok();
                } else {
                    led_1.set_high().ok();
                    led_2.set_low().ok();
                }
                toggle = !toggle;
                start = timer.get_ticks();
            } else {
                cortex_m::asm::nop();
            }
        }
    }
}

fn usb_poll(usb_dev: &mut UsbDeviceKey, keyboard: &mut UsbClassKey) {
    if usb_dev.poll(&mut [keyboard]) {
        keyboard.poll();
    }
}

// Disable NFC functionality on pins P0.09 and P0.10, allowing them
// to be used as regular GPIOs.
//
// Will cause a reset if it was necessary to modify the non-volatile
// NFC settings register
fn disable_nfc_pins(periphs: &nrf52840_hal::pac::Peripherals) {
    if periphs.UICR.nfcpins.read().protect().is_nfc() {
        // Enable erase
        periphs.NVMC.config.write(|w| w.wen().een());
        while periphs.NVMC.ready.read().ready().is_busy() {}

        // Erase regout0 page
        periphs
            .NVMC
            .erasepage()
            .write(|w| unsafe { w.erasepage().bits(&periphs.UICR.nfcpins as *const _ as u32) });
        while periphs.NVMC.ready.read().ready().is_busy() {}

        // enable write
        periphs.NVMC.config.write(|w| w.wen().wen());
        while periphs.NVMC.ready.read().ready().is_busy() {}

        // Set NFC settings
        periphs.UICR.nfcpins.modify(|_r, w| w.protect().disabled());
        while periphs.NVMC.ready.read().ready().is_busy() {}

        // Return UCIR to read only
        periphs.NVMC.config.write(|w| w.wen().ren());
        while periphs.NVMC.ready.read().ready().is_busy() {}

        // system reset
        SCB::sys_reset();
    }
}
