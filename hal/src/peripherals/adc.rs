use core::{marker::PhantomData, mem::ManuallyDrop};

use atsamd_hal_macros::{hal_cfg, hal_module};
use atsame51j::Mclk;
use seq_macro::seq;

use crate::{pac, gpio::AnyPin, typelevel::{NoneT, Sealed}};

#[hal_module(
    any("adc-d11", "adc-d21") => "adc/d11/mod.rs",
    "adc-d5x" => "adc/d5x/mod.rs",
)]
mod impls {}

pub use crate::adc_settings::*;

use super::{calibration, clock::Adc0Clock};

/// Marker type that represents an ADC channel capable of doing async
/// operations.
#[cfg(feature = "async")]
pub enum AdcFuture {}

/// Trait representing a GPIO pin which can be used as an input for ADC0
pub trait Adc0Pin: AnyPin + Sealed {
    const CHANNEL: u8;
}

/// Trait representing a GPIO pin which can be used as an input for ADC1
pub trait Adc1Pin: AnyPin + Sealed {
    const CHANNEL: u8;
}

pub struct Adc0Channel {
    id: u8
}

impl Adc0Channel {

    pub fn new<Id: Adc0Pin>(_pin: Id) -> Self {
        Self {
            id: Id::CHANNEL
        }
    }

    fn select(&self, adc: &mut Adc0) {
        while adc.adc.syncbusy().read().inputctrl().bit_is_set() {}
        adc.adc.inputctrl().modify(|_, w| unsafe{ w.muxpos().bits(self.id) });
    }

    pub fn read_blocking(&self, adc: &mut Adc0) -> u16 {
        self.select(adc);
        adc.power_up();
        adc.start_conversion();
        while adc.adc.intflag().read().resrdy().bit_is_clear() {}
        let res = adc.adc.result().read().result().bits();
        adc.power_down();
        res
    }

    pub fn read_buffer_blocking(&self, _adc: &mut Adc0, dst: &mut [u16]) {
        todo!()
    }

    #[cfg(feature="async")]
    pub async fn read(&self, _adc: &mut Adc0) -> u16 {
        todo!()
    }

    #[cfg(feature="async")]
    pub async fn read_buffer(&self, _adc: &mut Adc0, dst: &mut [u16]) {
        todo!()
    }
}

pub struct Adc0 {
    adc: pac::Adc0
}

impl Adc0 {
    pub fn new(adc0: pac::Adc0, settings: AdcSettingsBuilder, mclk: &mut Mclk, _clock: Adc0Clock) -> Self {
        mclk.apbdmask().modify(|_, w| w.adc0_().set_bit());

        // Calibrate and setup the Vref (This is done once)
        adc0.calib().write(|w| unsafe {
            w.biascomp().bits(calibration::adc0_biascomp_scale_cal());
            w.biasrefbuf().bits(calibration::adc0_biasref_scale_cal());
            w.biasr2r().bits(calibration::adc0_biasr2r_scale_cal())
        });

        let mut new_adc = Self {
            adc: adc0
        };

        new_adc.configure(settings);
        new_adc
    }

    pub fn configure(&mut self, settings: AdcSettingsBuilder) {
        self.adc.ctrla().modify(|_, w| match settings.clk_divider {
            AdcDivider::Div2 => w.prescaler().div2(),
            AdcDivider::Div4 => w.prescaler().div4(),
            AdcDivider::Div8 => w.prescaler().div8(),
            AdcDivider::Div16 => w.prescaler().div16(),
            AdcDivider::Div32 => w.prescaler().div32(),
            AdcDivider::Div64 => w.prescaler().div64(),
            AdcDivider::Div128 => w.prescaler().div128(),
            AdcDivider::Div256 => w.prescaler().div256(),
        });
        self.adc.ctrlb().modify(|_, w| match settings.bit_width {
            AdcBitWidth::Eight => w.ressel()._8bit(),
            AdcBitWidth::Ten => w.ressel()._10bit(),
            AdcBitWidth::Twelve => w.ressel()._12bit(),
        });
        while self.adc.syncbusy().read().ctrlb().bit_is_set() {}

        self.adc.sampctrl().modify(|_, w| unsafe {w.samplen().bits(settings.sample_clock_cycles)}); // sample length
        while self.adc.syncbusy().read().sampctrl().bit_is_set() {}
        self.adc.inputctrl().modify(|_, w| w.muxneg().gnd()); // No negative input (internal gnd)
        while self.adc.syncbusy().read().inputctrl().bit_is_set() {}
    }

    fn power_up(&mut self) {
        while self.adc.syncbusy().read().enable().bit_is_set() {}
        self.adc.ctrla().modify(|_, w| w.enable().set_bit());
        while self.adc.syncbusy().read().enable().bit_is_set() {}
    }

    fn power_down(&mut self) {
        while self.adc.syncbusy().read().enable().bit_is_set() {}
        self.adc.ctrla().modify(|_, w| w.enable().clear_bit());
        while self.adc.syncbusy().read().enable().bit_is_set() {}
    }

    #[inline(always)]
    fn start_conversion(&mut self) {
        // start conversion
        self.adc.swtrig().modify(|_, w| w.start().set_bit());
        // do it again because the datasheet tells us to
        self.adc.swtrig().modify(|_, w| w.start().set_bit());
    }

    fn enable_freerunning(&mut self) {
        self.adc.ctrlb().modify(|_, w| w.freerun().set_bit());
        while self.adc.syncbusy().read().ctrlb().bit_is_set() {}
    }

    fn disable_freerunning(&mut self) {
        self.adc.ctrlb().modify(|_, w| w.freerun().set_bit());
        while self.adc.syncbusy().read().ctrlb().bit_is_set() {}
    }

    /// Enables an interrupt when conversion is ready.
    fn enable_interrupts(&mut self) {
        self.adc.intflag().write(|w| w.resrdy().set_bit());
        self.adc.intenset().write(|w| w.resrdy().set_bit());
    }

    /// Disables the interrupt for when conversion is ready.
    fn disable_interrupts(&mut self) {
        self.adc.intenclr().write(|w| w.resrdy().set_bit());
    }
}