#![no_std]
#![no_main]

extern crate stm32g0xx_hal as hal;
use core::convert::Infallible;

use splitpea_bringup as _;

use hal::prelude::*;
use hal::stm32;
use hal::timer::Timer;
use stm32g0xx_hal::{rcc::{Config, PllConfig, Prescaler, RccExt}, gpio::{GpioExt, gpiob::PB, Output, PushPull, gpioa::PA, Input, Floating}, spi::{Spi, NoSck, NoMiso}, time::U32Ext, prelude::{PinState, OutputPin, InputPin}};
use heapless::spsc::{Queue, Producer, Consumer};

pub enum AnyPin<T> {
    PortA(PA<T>),
    PortB(PB<T>),
}

pub struct Msg {
    key_idx: u8,
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

#[rtic::app(device = hal::stm32, peripherals = true)]
mod app {
    use hal::{stm32::SPI1, gpio::{gpioa::PA2, Analog}};
    use smart_leds::{colors, SmartLedsWrite, gamma};
    use ws2812_spi::{Ws2812, MODE};

    use super::*;

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
        let mut smartled = Ws2812::new(smartled_spi);

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

        (
            Shared {},
            Local {
                timer,
                keys: Keys { rows, cols },
                state: [false; 20],
                sq_tx,
                sq_rx,
                smartled,
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

                let kidx = if (ridx & 1) == 0 {
                    (ridx * 5) + cidx
                } else {
                    ((ridx + 1) * 5) - 1 - cidx
                };

                let active = matches!(col.is_low(), Ok(true));
                if active != state[kidx] {
                    ctx.local.sq_tx.enqueue(Msg { key_idx: kidx as u8, state: active }).ok();
                }

                state[kidx] = active;
            }
            row.set_high().ok();
        }


    }

    #[idle(local = [sq_rx, smartled])]
    fn idle(ctx: idle::Context) -> ! {
        let mut lcl_state = [false; 20];
        let mut colors = [colors::BLACK; 20];
        let mut c = 0u8;

        let smartled = ctx.local.smartled;
        let _ = defmt::unwrap!(smartled.write(gamma(colors.iter().cloned())).map_err(drop));

        loop {
            let mut dirty = false;
            while let Some(msg) = ctx.local.sq_rx.dequeue() {
                dirty = true;

                let kiu = msg.key_idx as usize;

                lcl_state[kiu] = msg.state;

                if msg.state {
                    colors[kiu] = colors::RED;
                    let center_col = match c & 0b11 {
                        0 => colors::RED,
                        1 => colors::GREEN,
                        2 => colors::BLUE,
                        _ => colors::WHITE,
                    };
                    c = c.wrapping_add(1);
                    colors[17] = center_col;
                } else {
                    colors[kiu] = colors::BLACK;
                }
            }

            if dirty {

                let _ = defmt::unwrap!(smartled.write(gamma(colors.iter().cloned())).map_err(drop));
            }
        }
    }
}
