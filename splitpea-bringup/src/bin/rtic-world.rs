#![no_std]
#![no_main]

use hal::i2c_periph::I2CPeripheral;
use heapless::Vec;
use stm32g0xx_hal as hal;
use core::convert::Infallible;

use splitpea_bringup as _;

use hal::prelude::*;
use hal::stm32;
use hal::timer::Timer;
use hal::stm32::I2C1;
use stm32g0xx_hal::{rcc::{Config, PllConfig, Prescaler, RccExt}, gpio::{GpioExt, gpiob::PB, gpioc::PC, Output, PushPull, gpioa::PA, Input, Floating}, spi::{Spi, NoSck, NoMiso}, time::U32Ext, prelude::{PinState, OutputPin, InputPin}};
use heapless::spsc::{Queue, Producer, Consumer};
use heapless::Deque;
use serde::{Serialize, Deserialize};

pub enum AnyPin<T> {
    PortA(PA<T>),
    PortB(PB<T>),
}

pub struct Msg {
    key_idx: u8,
    led_idx: u8,
    state: bool,
}

impl<T> InputPin for AnyPin<Input<T>> {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        match self {
            AnyPin::PortA(ap) => ap.is_high(),
            AnyPin::PortB(bp) => bp.is_high(),
        }
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        match self {
            AnyPin::PortA(ap) => ap.is_low(),
            AnyPin::PortB(bp) => bp.is_low(),
        }
    }
}

impl OutputPin for AnyPin<Output<PushPull>> {
    type Error = Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        match self {
            AnyPin::PortA(ap) => ap.set_low(),
            AnyPin::PortB(bp) => bp.set_low(),
        }
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        match self {
            AnyPin::PortA(ap) => ap.set_high(),
            AnyPin::PortB(bp) => bp.set_high(),
        }
    }
}


pub struct Keys {
    pub cols: [AnyPin<Input<Floating>>; 5],
    pub rows: [AnyPin<Output<PushPull>>; 4],
}

#[derive(defmt::Format, Serialize, Deserialize)]
pub struct Event {
    kidx: u8,
    kind: EventKind,
}

#[derive(defmt::Format, Serialize, Deserialize)]
pub enum EventKind {
    KeyPress,
    KeyRelease,
    // TODO: KeyHold?
}

#[rtic::app(device = hal::stm32, peripherals = true)]
mod app {
    use hal::{stm32::SPI1, gpio::{gpioa::PA2, Analog}, i2c_periph::I2CPeripheral};
    use smart_leds::{colors, SmartLedsWrite, gamma};
    use ws2812_spi::{Ws2812, MODE};

    use super::*;
    use cassette::{Cassette, pin_mut};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        timer: Timer<stm32::TIM17>,
        keys: Keys,
        state: [bool; 20],
        sq_tx: Producer<'static, Msg, 32>,
        sq_rx: Consumer<'static, Msg, 32>,
        smartled: Ws2812<Spi<SPI1, (NoSck, NoMiso, PA2<Analog>)>>,
        i2c: I2CPeripheral<I2C1>,
    }

    #[init(local = [state_q: Queue<Msg, 32> = Queue::new()])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let config = Config::pll()
            .pll_cfg(PllConfig::with_hsi(1, 8, 2))
            .ahb_psc(Prescaler::NotDivided)
            .apb_psc(Prescaler::NotDivided);
        let mut rcc = ctx.device.RCC.freeze(config);

        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);
        let gpioc = ctx.device.GPIOC.split(&mut rcc);

        let mut timer = ctx.device.TIM17.timer(&mut rcc);
        timer.start(100.hz());
        timer.listen();

        let (sq_tx, sq_rx) = ctx.local.state_q.split();

        let mut smartled_spi = Spi::spi1(
            ctx.device.SPI1,
            (NoSck, NoMiso, gpioa.pa2),
            MODE,
            3_800_000u32.hz(),
            &mut rcc,
        );
        smartled_spi.half_duplex_output_enable(true);
        let smartled = Ws2812::new(smartled_spi);

        // Active High
        let _led_enable = gpiob.pb3.into_push_pull_output_in_state(PinState::High);

        let cols = [
            AnyPin::PortB(gpiob.pb0.into_floating_input().downgrade()),
            AnyPin::PortA(gpioa.pa11.into_floating_input().downgrade()),
            AnyPin::PortA(gpioa.pa12.into_floating_input().downgrade()),
            AnyPin::PortA(gpioa.pa1.into_floating_input().downgrade()),
            AnyPin::PortA(gpioa.pa0.into_floating_input().downgrade()),
        ];

        let rows = [
            AnyPin::PortA(gpioa.pa4.into_push_pull_output().downgrade()),
            AnyPin::PortA(gpioa.pa5.into_push_pull_output().downgrade()),
            AnyPin::PortA(gpioa.pa6.into_push_pull_output().downgrade()),
            AnyPin::PortA(gpioa.pa7.into_push_pull_output().downgrade()),
        ];

        let _unused = gpioc.pc14;
        let _unused = gpiob.pb7;

        let i2c = I2CPeripheral::new(
            ctx.device.I2C1,
            gpiob.pb9.into_open_drain_output(),
            gpiob.pb8.into_open_drain_output(),
            &mut rcc,
            0x42,
        );

        (
            Shared {},
            Local {
                timer,
                keys: Keys { rows, cols },
                state: [false; 20],
                sq_tx,
                sq_rx,
                smartled,
                i2c,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = TIM17, local = [timer, state, keys, sq_tx])]
    fn timer_tick(ctx: timer_tick::Context) {
        ctx.local.timer.clear_irq();
        let state = ctx.local.state;
        let keys = ctx.local.keys;

        for (ridx, row) in keys.rows.iter_mut().enumerate() {
            row.set_low().ok();

            // Let the row/column lines settle
            cortex_m::asm::delay(1_000);

            for (cidx, col) in keys.cols.iter().enumerate() {

                let kidx = (ridx * 5) + cidx;

                let lidx = if (ridx & 1) == 0 {
                    kidx
                } else {
                    ((ridx + 1) * 5) - 1 - cidx
                };

                let active = matches!(col.is_low(), Ok(true));
                if active != state[kidx] {
                    ctx.local.sq_tx.enqueue(Msg { key_idx: kidx as u8, led_idx: lidx as u8, state: active }).ok();
                }

                state[kidx] = active;
            }
            row.set_high().ok();
        }


    }

    #[idle(local = [sq_rx, smartled,  i2c])]
    fn idle(ctx: idle::Context) -> ! {
        let mut lcl_state = [false; 20];
        let mut colors = [colors::BLACK; 20];
        let mut c = 0u8;

        // TODO: Make this a SPSC queue and give receiver to async
        let mut events: Queue<Event, 32> = Queue::new();
        let (mut prod, cons) = events.split();

        let smartled = ctx.local.smartled;
        let _ = defmt::unwrap!(smartled.write(gamma(colors.iter().cloned())).map_err(drop));

        let mut i2cwerk = I2cWerk {
            i2c: ctx.local.i2c,
            cons,
            hold: Deque::new(),
        };
        let werkfut = i2cwerk.werk();
        pin_mut!(werkfut);

        let mut werk_cas = Cassette::new(werkfut);

        loop {
            let mut dirty = false;
            let _ = werk_cas.poll_on();

            while let Some(msg) = ctx.local.sq_rx.dequeue() {
                dirty = true;

                let kiu = msg.key_idx as usize;
                let liu = msg.led_idx as usize;

                let changed = lcl_state[kiu] != msg.state;
                lcl_state[kiu] = msg.state;

                let _ = match (changed, msg.state) {
                    (true, true) => prod.enqueue(Event { kidx: msg.key_idx, kind: EventKind::KeyPress }),
                    (true, false) => prod.enqueue(Event { kidx: msg.key_idx, kind: EventKind::KeyRelease }),
                    (false, _) => Ok(()),
                };

                if msg.state {
                    colors[liu] = colors::RED;
                    defmt::println!("kidx: {=u8}, lidx: {=u8}", msg.key_idx, msg.led_idx);
                    let center_col = match c & 0b11 {
                        0 => colors::RED,
                        1 => colors::GREEN,
                        2 => colors::BLUE,
                        _ => colors::WHITE,
                    };
                    c = c.wrapping_add(1);
                    colors[17] = center_col;
                } else {
                    colors[liu] = colors::BLACK;
                }
            }

            if dirty {
                let _ = defmt::unwrap!(smartled.write(gamma(colors.iter().cloned())).map_err(drop));
            }
        }
    }
}

struct I2cWerk<'a> {
    i2c: &'static mut I2CPeripheral<I2C1>,
    cons: Consumer<'a, Event, 32>,
    hold: Deque<Event, 32>,
}

impl<'a> I2cWerk<'a> {
    // pub async fn werk(&mut self) -> ! {
    //     loop {
    //         defmt::println!("Waiting for match...");
    //         self.i2c.match_address_read().await;
    //         defmt::println!("Match!");
    //         for i in 0..10 {
    //             self.i2c.send_read_byte(i).await;
    //             defmt::println!("Sent {=u8}", i);
    //         }
    //         defmt::println!("Waiting for stop...");
    //         self.i2c.wait_for_stop().await;
    //     }
    // }

    pub async fn werk(&mut self) -> ! {
        loop {
            defmt::println!("Waiting for match...");

            // Wait for a "write command" over I2C
            self.i2c.match_address_write().await;

            // Quickly drain any incoming messages
            // TODO: Hmmm, this will essential reverse the order, because
            // a vec is basically a stack, and I really want "pop front"...
            // I guess I probably want a circular buffer of some sort instead...
            while let Some(event) = self.cons.dequeue() {
                self.hold.push_back(event).ok();
            }

            match self.i2c.get_written_byte().await {
                // Get depth of current queue
                0x10 => {
                    let len = self.hold.len();
                    let len_u8 = len as u8;

                    defmt::println!("Got qlen command!, {=u8} ready", len_u8);
                    defmt::println!("Wait for read cmd...");
                    self.i2c.match_address_read().await;
                    defmt::println!("Reading...");
                    self.i2c.send_read_byte(len_u8).await;
                    defmt::println!("Read one byte! Waiting for stop...");
                    self.i2c.wait_for_stop().await;
                },

                // Get number of events (not bytes! 2 bytes each!)
                0x20 => {
                    let ct = self.i2c.get_written_byte().await as usize;
                    let len = self.hold.len();
                    defmt::assert!(ct <= len);
                    self.i2c.match_address_read().await;

                    for _ in 0..ct {
                        let data = defmt::unwrap!(self.hold.pop_front());
                        let mut buf = [0u8; 2];
                        defmt::unwrap!(postcard::to_slice(&data, &mut buf).map_err(drop));
                        self.i2c.send_read_byte(buf[0]).await;
                        self.i2c.send_read_byte(buf[1]).await;
                    }
                    self.i2c.wait_for_stop().await;
                }
                _ => defmt::unimplemented!(),
            }
            // defmt::println!("Match!");
            // for i in 0..10 {
            //     self.i2c.send_read_byte(i).await;
            //     defmt::println!("Sent {=u8}", i);
            // }
            // defmt::println!("Waiting for stop...");
            // self.i2c.wait_for_stop().await;
        }
    }
}
