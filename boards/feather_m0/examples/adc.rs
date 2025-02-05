#![no_std]
#![no_main]

// #[cfg(not(feature = "use_semihosting"))]
// use panic_halt as _;
// #[cfg(feature = "use_semihosting")]
// use panic_semihosting as _;

use defmt_rtt as _;
use panic_probe as _;

use cortex_m_semihosting::hprintln;

use bsp::hal;
use bsp::pac;
use feather_m0 as bsp;

use bsp::entry;
use hal::adc::{Accumulation, Adc, Config, Prescaler, Resolution};
use hal::clock::GenericClockController;
use hal::prelude::*;
use pac::{CorePeripherals, Peripherals};

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.gclk,
        &mut peripherals.pm,
        &mut peripherals.sysctrl,
        &mut peripherals.nvmctrl,
    );
    let pins = bsp::Pins::new(peripherals.port);
    let mut delay = hal::delay::Delay::new(core.SYST, &mut clocks);

    let gclk0 = clocks.gclk0();
    let adc_clock = clocks.adc(&gclk0).unwrap();

    let adc_config = Config::new()
        .clock_cycles_per_sample(5)
        .clock_divider(Prescaler::Div4)
        .sample_resolution(Resolution::_12bit)
        .accumulation_method(Accumulation::Single);

    let mut adc = Adc::new(peripherals.adc, adc_config, &mut peripherals.pm, &adc_clock).unwrap();
    let mut a0: bsp::A0 = pins.a0.into();

    loop {
        let data = adc.read_blocking(&mut a0);
        defmt::info!("{}", data);
        delay.delay_ms(1000u16);
    }
}
