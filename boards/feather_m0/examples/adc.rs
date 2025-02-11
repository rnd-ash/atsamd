#![no_std]
#![no_main]

#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use cortex_m_semihosting::hprintln;

use bsp::hal;
use bsp::pac;
use feather_m0 as bsp;

use bsp::entry;
use hal::adc::{Accumulation, Adc, Adc0, Config, Prescaler, Resolution};
use hal::clock::GenericClockController;
use hal::prelude::*;
use pac::{CorePeripherals, Peripherals};

atsamd_hal::bind_interrupts!(struct Irqs {
    ADC => atsamd_hal::adc::InterruptHandler<Adc0>;
});

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
        .clock_divider(Prescaler::Div8)
        .sample_resolution(Resolution::_12bit)
        .accumulation_method(Accumulation::Single);

    let mut adc = Adc::new(peripherals.adc, adc_config, &mut peripherals.pm, &adc_clock)
        .unwrap()
        .into_future(Irqs);
    let mut a0 = pins.a0.into_alternate();

    loop {
        let mut buf = [0; 16];
        let data = adc.read_buffer_blocking(&mut a0, &mut buf);
        hprintln!("buf: {}", buf);
        delay.delay_ms(1000u16);
    }
}
