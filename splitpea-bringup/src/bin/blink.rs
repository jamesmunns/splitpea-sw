#![no_main]
#![no_std]

use core::convert::Infallible;

use splitpea_bringup as _; // global logger + panicking-behavior + memory layout

use stm32g0xx_hal::{stm32, rcc::{Config, PllConfig, Prescaler, RccExt}, gpio::{GpioExt, gpiob::PB, Output, PushPull, gpioa::PA, Input, Floating}, spi::{Spi, NoSck, NoMiso}, time::U32Ext, prelude::{PinState, OutputPin, InputPin}};
use ws2812_spi::{MODE, Ws2812};
use smart_leds::{SmartLedsWrite, colors, gamma};

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Hello, world!");

    nmmain();

    splitpea_bringup::exit()
}

fn nmmain() {

    let board = defmt::unwrap!(stm32::Peripherals::take());
    let core = defmt::unwrap!(stm32::CorePeripherals::take());

    // Configure clocks
    let config = Config::pll()
        .pll_cfg(PllConfig::with_hsi(1, 8, 2))
        .ahb_psc(Prescaler::NotDivided)
        .apb_psc(Prescaler::NotDivided);
    let mut rcc = board.RCC.freeze(config);

    let gpioa = board.GPIOA.split(&mut rcc);
    let gpiob = board.GPIOB.split(&mut rcc);
    let gpioc = board.GPIOC.split(&mut rcc);

    defmt::println!("ding!");


    let mut smartled_spi = Spi::spi1(
        board.SPI1,
        (NoSck, NoMiso, gpioa.pa2),
        MODE,
        3_800_000u32.hz(),
        &mut rcc,
    );
    smartled_spi.half_duplex_output_enable(true);
    let mut smartled = Ws2812::new(smartled_spi);

    // Active High
    let _led_enable = gpiob.pb3.into_push_pull_output_in_state(PinState::High);
    // let _led_enable = gpiob.pb4.into_push_pull_output_in_state(PinState::High);
    // let _led_enable = gpiob.pb5.into_push_pull_output_in_state(PinState::High);
    // let _led_enable = gpiob.pb6.into_push_pull_output_in_state(PinState::High);

    cortex_m::asm::delay(1_000_000);

    defmt::println!("dong!");

    // for _ in 0..100 {
    //     for col in [colors::RED, colors::GREEN, colors::BLUE, colors::BLACK] {
    //         defmt::println!("doot! {=u8} {=u8} {=u8}", col.r, col.g, col.b);

    //         let colores = [col; 20];
    //         let _ = defmt::unwrap!(smartled.write(gamma(colores.iter().cloned())).map_err(drop));

    //         cortex_m::asm::delay(128_000_000);
    //     }
    // }

    let cols = &mut [
        AnyPin::PortB(gpiob.pb0.into_floating_input().downgrade()),
        AnyPin::PortA(gpioa.pa11.into_floating_input().downgrade()),
        AnyPin::PortA(gpioa.pa12.into_floating_input().downgrade()),
        AnyPin::PortA(gpioa.pa1.into_floating_input().downgrade()),
        AnyPin::PortA(gpioa.pa0.into_floating_input().downgrade()),
    ];

    let rows = &mut [
        AnyPin::PortA(gpioa.pa4.into_push_pull_output().downgrade()),
        AnyPin::PortA(gpioa.pa5.into_push_pull_output().downgrade()),
        AnyPin::PortA(gpioa.pa6.into_push_pull_output().downgrade()),
        AnyPin::PortA(gpioa.pa7.into_push_pull_output().downgrade()),
    ];

    for row in rows.iter_mut() {
        row.set_high().ok();
    }

    let mut c = 0u8;

    loop {
        for (ridx, row) in rows.iter_mut().enumerate() {
            row.set_low().ok();
            for (cidx, col) in cols.iter().enumerate() {
                cortex_m::asm::delay(32_000);

                if let Ok(true) = col.is_low() {
                    defmt::println!("r: {=usize}, c: {=usize}", ridx, cidx);
                    let mut colors = [colors::BLACK; 20];

                    let idx = if (ridx & 1) == 0 {
                        (ridx * 5) + cidx
                    } else {
                        ((ridx + 1) * 5) - 1 - cidx
                    };

                    let center_col = match c & 0b11 {
                        0 => colors::RED,
                        1 => colors::GREEN,
                        2 => colors::BLUE,
                        _ => colors::WHITE,
                    };
                    c = c.wrapping_add(1);

                    colors[idx] = colors::WHITE;
                    let _ = defmt::unwrap!(smartled.write(gamma(colors.iter().cloned())).map_err(drop));

                    while let Ok(true) = col.is_low() { }
                }
            }
            row.set_high().ok();
        }
    }
}

enum AnyPin<T> {
    PortA(PA<T>),
    PortB(PB<T>),
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
