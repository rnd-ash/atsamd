use core::{marker::PhantomData, ops::Deref};

use atsamd_hal_macros::{hal_cfg, hal_module};
use atsame51j::Peripherals;
use pac::Mclk;
use seq_macro::seq;

use crate::{
    gpio::AnyPin,
    pac,
    typelevel::{NoneT, Sealed},
};

#[hal_module(
    any("adc-d11", "adc-d21") => "d11/mod.rs",
    "adc-d5x" => "d5x/mod.rs",
)]
mod impls {}
mod adc_settings;

pub use adc_settings::*;

use super::{
    calibration,
    clock::{self, Adc0Clock},
};

/// Marker type that represents an ADC channel capable of doing async
/// operations.
#[cfg(feature = "async")]
pub enum AdcFuture {}

/// Trait representing an ADC instance
pub trait AdcInstance {
    #[cfg(feature = "async")]
    type Interrupt: crate::async_hal::interrupts::InterruptSource;

    // The Adc0 and Adc1 PAC types implement Deref
    type Instance: Deref<Target = pac::adc0::RegisterBlock>;
    type Clock;

    fn reg_block(&self) -> &pac::adc0::RegisterBlock;
    fn peripheral_reg_block(p: &mut Peripherals) -> &pac::adc0::RegisterBlock;
    fn enable_mclk(mclk: &mut Mclk);
    fn calibrate(instance: &Self::Instance);

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
}

// TODO: The next few lines will need to be adjusted for SAMD11 and SAMD21: they only have 1 ADC
pub struct Adc0 {
    adc: pac::Adc0,
}
impl AdcInstance for Adc0 {
    type Instance = pac::Adc0;
    type Clock = clock::Adc0Clock;

    #[cfg(feature = "async")]
    type Interrupt = crate::async_hal::interrupts::ADC0;

    fn reg_block(&self) -> &pac::adc0::RegisterBlock {
        &self.adc
    }

    fn peripheral_reg_block(p: &mut Peripherals) -> &pac::adc0::RegisterBlock {
        &p.adc0
    }

    fn enable_mclk(mclk: &mut Mclk) {
        mclk.apbdmask().modify(|_, w| w.adc0_().set_bit());
    }

    fn calibrate(instance: &Self::Instance) {
        instance.calib().write(|w| unsafe {
            w.biascomp().bits(calibration::adc0_biascomp_scale_cal());
            w.biasrefbuf().bits(calibration::adc0_biasref_scale_cal());
            w.biasr2r().bits(calibration::adc0_biasr2r_scale_cal())
        });
    }

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        &impls::async_api::ADC_WAKERS[0]
    }
}

pub struct Adc1 {
    adc: pac::Adc1,
}

impl AdcInstance for Adc1 {
    type Instance = pac::Adc1;
    type Clock = clock::Adc1Clock;

    #[cfg(feature = "async")]
    type Interrupt = crate::async_hal::interrupts::ADC1;

    fn reg_block(&self) -> &pac::adc0::RegisterBlock {
        &self.adc
    }

    fn peripheral_reg_block(p: &mut Peripherals) -> &pac::adc0::RegisterBlock {
        &p.adc1
    }

    fn enable_mclk(mclk: &mut Mclk) {
        mclk.apbdmask().modify(|_, w| w.adc1_().set_bit());
    }
    fn calibrate(instance: &Self::Instance) {
        instance.calib().write(|w| unsafe {
            w.biascomp().bits(calibration::adc1_biascomp_scale_cal());
            w.biasrefbuf().bits(calibration::adc1_biasref_scale_cal());
            w.biasr2r().bits(calibration::adc1_biasr2r_scale_cal())
        });
    }

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        &impls::async_api::ADC_WAKERS[1]
    }
}

/// Trait representing a GPIO pin which can be used as an input for an ADC
pub trait AdcPin<I, C>: AnyPin + Sealed
where
    I: AdcInstance,
    C: ChId,
{
    type Configured;

    fn into_function(self) -> Self::Configured;
}

/// Trait representing an ADC channel ID.
pub trait ChId {
    const ID: u8;
}

/// ADC channel.
///
/// This struct must hold a concrete [`Pin`](crate::gpio::Pin) which implements
/// [`AdcPin`] in order to perform conversions. By default, channels don't hold any pin when they are created by [`Adc::new`]. Use [`Channel::with_pin`](Self::with_pin) to give a pin to this [`Channel`].
pub struct Channel<I: AdcInstance, Id: ChId, P> {
    _pin: P,
    _instance: PhantomData<I>,
    _id: PhantomData<Id>,
}

// These methods are only implemented for a Channel that doesn't hold a pin yet
impl<I: AdcInstance, Id: ChId> Channel<I, Id, NoneT> {
    // NOTE: `new`` must be private so a channel isn't accidentally created outside this
    // module, breaking the typelevel guarantees laid out by the adc driver
    fn new() -> Channel<I, Id, NoneT> {
        Channel {
            _pin: NoneT,
            _instance: PhantomData,
            _id: PhantomData,
        }
    }

    /// Give a concrete pin to this [`Channel`], which will be used by the ADC
    /// to measure voltage.
    ///
    /// This methods accepts any pin that can potentially be configured as an
    /// ADC channel, and automatically puts it in the Alternate B mode.
    pub fn with_pin<N: AdcPin<I, Id>>(self, pin: N) -> Channel<I, Id, N::Configured> {
        // NOTE: While AdcPin is implemented for any pin that has the *potential* to be
        // turned into an AlternateB pin (which is the ADC function), we know that any
        // Channel holding a type implementing AdcPin must have already configured the
        // pin to the alternate B function, since the with_pin method is the only way to insert a
        // pin into the Channel.
        Channel {
            _pin: pin.into_function(),
            _instance: PhantomData,
            _id: PhantomData,
        }
    }
}

// These methods are only implemented for a Channel that holds a configured pin
impl<I: AdcInstance, Id: ChId, P: AdcPin<I, Id>> Channel<I, Id, P> {
    pub fn read_blocking(&self, adc: &mut Adc<I>) -> u16 {
        self.select(adc);
        adc.power_up();
        adc.start_conversion();
        while adc.adc.intflag().read().resrdy().bit_is_clear() {}
        let res = adc.adc.result().read().result().bits();
        adc.power_down();
        res
    }

    pub fn read_buffer_blocking(&self, _adc: &mut Adc<I>, dst: &mut [u16]) {
        todo!()
    }

    #[cfg(feature = "async")]
    pub async fn read(&self, _adc: &mut Adc<I>) -> u16 {
        todo!()
    }

    #[cfg(feature = "async")]
    pub async fn read_buffer(&self, _adc: &mut Adc<I>, dst: &mut [u16]) {
        todo!()
    }

    fn select(&self, adc: &mut Adc<I>) {
        let adc = adc.reg_block();

        while adc.syncbusy().read().inputctrl().bit_is_set() {}
        adc.inputctrl()
            .modify(|_, w| unsafe { w.muxpos().bits(Id::ID) });
    }
}

pub struct Adc<I: AdcInstance> {
    adc: I::Instance,
}

impl<I: AdcInstance> Adc<I> {
    pub fn new(
        adc: I::Instance,
        settings: AdcSettingsBuilder,
        mclk: &mut Mclk,
        _clock: I::Clock,
    ) -> (Self, Channels<I>) {
        I::enable_mclk(mclk);
        // Calibrate and setup the Vref (This is done once)
        let mut new_adc = Self { adc };

        new_adc.configure(settings);
        I::calibrate(&new_adc.adc);

        (new_adc, Channels::new())
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

        self.adc
            .sampctrl()
            .modify(|_, w| unsafe { w.samplen().bits(settings.sample_clock_cycles) }); // sample length
        while self.adc.syncbusy().read().sampctrl().bit_is_set() {}
        self.adc.inputctrl().modify(|_, w| w.muxneg().gnd()); // No negative input (internal gnd)
        while self.adc.syncbusy().read().inputctrl().bit_is_set() {}
    }

    /// Access the struct's underlying PAC object
    fn reg_block(&self) -> &pac::adc0::RegisterBlock {
        &self.adc
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

// Here I declare the number of channels for the SAME51 ADC
// TODO: declare channels for other chip variants
#[hal_cfg(any("eic-d5x"))]
macro_rules! with_num_channels {
    ($some_macro:ident) => {
        $some_macro! {16}
    };
}

/// Get the number of channels as a literal
macro_rules! get {
    ($literal:literal) => {
        $literal
    };
}

/// The number of ADC channels per instance on this chip.
pub const NUM_CHANNELS: usize = with_num_channels!(get);

macro_rules! define_channels_struct {
    ($num_channels:literal) => {
        seq!(N in 0..$num_channels {
            #(
                /// Type alias for a channel number
                pub enum Ch~N {}

                impl ChId for Ch~N {
                    const ID: u8 = N;
                }
            )*

            /// Struct generating individual handles to each ADC channel
            pub struct Channels<I: AdcInstance>(
                #(
                    pub Channel<I, Ch~N, NoneT>,
                )*
            );

            impl<I: AdcInstance> Channels<I> {
                fn new() -> Self {
                    Self (
                        #(
                            Channel::new(),
                        )*
                    )
                }
            }
        });
    };
}

with_num_channels!(define_channels_struct);
