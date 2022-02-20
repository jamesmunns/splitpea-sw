#![no_main]
#![no_std]

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

type UsbClassKey = keyberon::Class<'static, Usbd<UsbPeripheral<'static>>, Leds>;
type UsbDeviceKey = usb_device::device::UsbDevice<'static, Usbd<UsbPeripheral<'static>>>;

pub struct Leds {
    //     caps_lock: gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>,
}

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
    use heapless::spsc::Queue;
    use stem_bringup::monotonic::{ExtU32, MonoTimer};
    use postcard::{to_vec_cobs, CobsAccumulator, FeedResult};
    use splitpea_driver::Splitpea;


    // Keyberon
    use keyberon::{
        action::{
            k,
            l,
            Action::{self, *},
        },
        debounce::Debouncer,
        hid::HidClass,
        // impl_heterogenous_array,
        key_code::{KbHidReport, KeyCode, KeyCode::*},
        layout::{CustomEvent, Event, Layout},
        matrix::{Matrix, PressedKeys},
    };

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
    }

    #[init(local = [
        usb_bus: Option<UsbBusAllocator<Usbd<UsbPeripheral<'static>>>> = None,
        // incoming: Queue<ToMcu, 8> = Queue::new(),
        // outgoing: Queue<Result<ToPc, ()>, 8> = Queue::new(),
        uart_rx_buf: [u8; 64] = [0; 64],
        uart_tx_buf: [u8; 1] = [0],
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

        usb_tick::spawn().ok();
        key_tick::spawn().ok();
        (
            Shared {},
            Local {
                // worker,
                // interface_comms,
                usb_class,
                usb_dev,
                pea,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [pea, ctr: u32 = 0])]
    fn key_tick(cx: key_tick::Context) {
        let pea = cx.local.pea;

        *cx.local.ctr += 1;
        if *cx.local.ctr >= 100 {
            *cx.local.ctr = 0;
            defmt::println!("KEY TICK");
        }

        let evts = defmt::unwrap!(pea.get_all_events().map_err(drop));

        for evt in evts {
            defmt::println!("EVENT: {:?}", evt);
        }

        key_tick::spawn_after(10.millis()).ok();
    }

    #[task(local = [usb_dev, usb_class, ctr: u32 = 0])]
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

        // if let Some(out) = interface_comms.to_pc.dequeue() {
        //     if let Ok(ser_msg) = to_vec_cobs::<_, 128>(&out) {
        //         usb_serial.write(&ser_msg).ok();
        //     } else {
        //         defmt::panic!("Serialization error!");
        //     }
        // }

        // match usb_serial.read(&mut buf) {
        //     Ok(sz) if sz > 0 => {
        //         // let buf = &buf[..sz];
        //         // let mut window = &buf[..];

        //         // 'cobs: while !window.is_empty() {
        //         //     window = match cobs_buf.feed::<phm_icd::ToMcu>(&window) {
        //         //         FeedResult::Consumed => break 'cobs,
        //         //         FeedResult::OverFull(new_wind) => new_wind,
        //         //         FeedResult::DeserError(new_wind) => new_wind,
        //         //         FeedResult::Success { data, remaining } => {
        //         //             defmt::println!("got: {:?}", data);
        //         //             interface_comms.to_mcu.enqueue(data).ok();
        //         //             remaining
        //         //         }
        //         //     };
        //         // }
        //     }
        //     Ok(_) | Err(usb_device::UsbError::WouldBlock) => {}
        //     Err(_e) => defmt::panic!("Usb Error!"),
        // }

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
