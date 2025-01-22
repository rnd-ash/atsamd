use core::{marker::PhantomData, ops::Deref};

use atsamd_hal_macros::{hal_cfg, hal_module};
use pac::{Mclk, Peripherals};
use seq_macro::seq;

use crate::{
    gpio::AnyPin,
    pac,
    time::Hertz,
    typelevel::{NoneT, Sealed},
};

#[hal_module(
    any("adc-d11", "adc-d21") => "d11/mod.rs",
    "adc-d5x" => "d5x/mod.rs",
)]
mod impls {}

#[cfg(feature = "async")]
mod async_api;
#[cfg(feature = "async")]
pub use async_api::*;

mod adc_settings;
pub use adc_settings::*;

use super::{calibration, clock};

use crate::pac::adc0;

pub use adc0::avgctrl::Samplenumselect;
/// Samples per reading
pub use adc0::avgctrl::Samplenumselect as SampleRate;
/// Clock frequency relative to the system clock
pub use adc0::ctrla::Prescalerselect as Prescaler;
/// Reading resolution in bits
pub use adc0::ctrlb::Resselselect as Resolution;
/// Reference voltage (or its source)
pub use adc0::refctrl::Refselselect as Reference;

/// Trait representing an ADC instance
pub trait AdcInstance {
    #[cfg(feature = "async")]
    type Interrupt: crate::async_hal::interrupts::InterruptSource;

    // The Adc0 and Adc1 PAC types implement Deref
    type Instance: Deref<Target = pac::adc0::RegisterBlock>;
    type Clock: Into<Hertz>;

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

    #[inline]
    fn peripheral_reg_block(p: &mut Peripherals) -> &pac::adc0::RegisterBlock {
        &p.adc0
    }

    #[inline]
    fn enable_mclk(mclk: &mut Mclk) {
        mclk.apbdmask().modify(|_, w| w.adc0_().set_bit());
    }

    #[inline]
    fn calibrate(instance: &Self::Instance) {
        instance.calib().write(|w| unsafe {
            w.biascomp().bits(calibration::adc0_biascomp_scale_cal());
            w.biasrefbuf().bits(calibration::adc0_biasref_scale_cal());
            w.biasr2r().bits(calibration::adc0_biasr2r_scale_cal())
        });
    }

    #[cfg(feature = "async")]
    #[inline]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        &async_api::ADC_WAKERS[0]
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

    #[inline]
    fn peripheral_reg_block(p: &mut Peripherals) -> &pac::adc0::RegisterBlock {
        &p.adc1
    }

    #[inline]
    fn enable_mclk(mclk: &mut Mclk) {
        mclk.apbdmask().modify(|_, w| w.adc1_().set_bit());
    }

    #[inline]
    fn calibrate(instance: &Self::Instance) {
        instance.calib().write(|w| unsafe {
            w.biascomp().bits(calibration::adc1_biascomp_scale_cal());
            w.biasrefbuf().bits(calibration::adc1_biasref_scale_cal());
            w.biasr2r().bits(calibration::adc1_biasr2r_scale_cal())
        });
    }

    #[cfg(feature = "async")]
    #[inline]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
        &async_api::ADC_WAKERS[1]
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
    #[inline]
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
    #[inline]
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
    #[inline]
    pub fn read_blocking(&self, adc: &mut Adc<I, NoneT>) -> u16 {
        //f(Id::ID as u16);
        adc.read_blocking(Id::ID)
    }

    #[inline]
    pub fn read_buffer_blocking(&self, adc: &mut Adc<I>, dst: &mut [u16]) {
        //adc.read_buffer_blocking(Id::ID)
        todo!()
    }

    #[cfg(feature = "async")]
    #[inline]
    pub async fn read<F>(&self, adc: &mut Adc<I, F>) -> u16
    where
        F: crate::async_hal::interrupts::Binding<I::Interrupt, async_api::InterruptHandler<I>>,
    {
        adc.read(Id::ID).await
    }

    #[cfg(feature = "async")]
    #[inline]
    pub async fn read_buffer(&self, _adc: &mut Adc<I>, dst: &mut [u16]) {
        todo!()
    }
}

/// ADC Instance
pub struct Adc<I: AdcInstance, F = NoneT> {
    adc: I::Instance,
    _irqs: PhantomData<F>,
}

pub struct AdcFuture;

impl<I: AdcInstance> Adc<I> {
    /// Construct a new ADC instance
    ///
    /// ## Important
    /// This function will return None (No ADC) if the clock source provided
    /// is faster than 100Mhz, since this is the maximum frequency for GCLK_ADCx as per
    /// the datasheet.
    ///
    /// NOTE: If you plan to run the chip up to 125C, then the maximum GCLK frequency for the ADC
    /// is restricted to 90Mhz for stable performance.
    #[inline]
    pub fn new(
        adc: I::Instance,
        settings: AdcSettingsBuilder,
        mclk: &mut Mclk,
        clock: I::Clock,
    ) -> Option<(Self, Channels<I>)> {
        if (clock.into() as Hertz).to_Hz() > 100_000_000 {
            // Clock source is too fast
            return None;
        }
        I::enable_mclk(mclk);

        let mut new_adc = Self {
            adc,
            _irqs: PhantomData,
        };

        // Reset ADC here as we cannot guarantee its state
        // This also disables the ADC
        new_adc.adc.ctrla().modify(|_, w| w.swrst().set_bit());
        new_adc.sync();

        I::calibrate(&new_adc.adc);
        new_adc.configure(settings);
        Some((new_adc, Channels::new()))
    }

    #[inline]
    pub fn configure(&mut self, settings: AdcSettingsBuilder) {
        // Disable ADC before we do anything!
        self.power_down();
        self.sync();
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
        self.sync();

        self.adc
            .sampctrl()
            .modify(|_, w| unsafe { w.samplen().bits(settings.sample_clock_cycles) }); // sample length
        self.sync();
        self.adc.inputctrl().modify(|_, w| w.muxneg().gnd()); // No negative input (internal gnd)
        self.sync();

        self.adc.avgctrl().modify(|_, w| {
            w.samplenum().variant(Samplenumselect::_1);
            unsafe { w.adjres().bits(0) }
        });
        self.sync();

        self.adc
            .refctrl()
            .modify(|_, w| w.refsel().variant(Reference::Intref));
        self.sync();
    }

    #[inline]
    pub fn read_blocking(&mut self, ch: u8) -> u16 {
        self.mux(ch);
        self.power_up();
        self.sync();
        self.start_conversion();
        while self.adc.intflag().read().resrdy().bit_is_clear() {}
        let res = self.result();
        self.power_down();
        res
    }

    #[inline]
    pub fn read_ptat_blocking(&mut self) -> u16 {
        self.read_blocking(0x19)
    }

    #[inline]
    pub fn read_ctat_blocking(&mut self) -> u16 {
        self.read_blocking(0x1A)
    }
}

impl<I: AdcInstance, T> Adc<I, T> {
    #[inline]
    fn sync(&self) {
        // Slightly more performant than checking the individual bits
        // since we avoid an extra instruction to bit shift
        while self.adc.syncbusy().read().bits() != 0 {}
    }

    #[inline]
    fn power_up(&mut self) {
        self.adc.ctrla().modify(|_, w| w.enable().set_bit());
        self.sync();
    }

    #[inline]
    fn power_down(&mut self) {
        self.adc.ctrla().modify(|_, w| w.enable().clear_bit());
        self.sync();
    }

    #[inline]
    fn start_conversion(&mut self) {
        self.adc.swtrig().modify(|_, w| w.start().set_bit());
        self.sync();
    }

    #[inline]
    fn enable_freerunning(&mut self) {
        self.adc.ctrlb().modify(|_, w| w.freerun().set_bit());
        while self.adc.syncbusy().read().ctrlb().bit_is_set() {}
    }

    #[inline]
    fn disable_freerunning(&mut self) {
        self.adc.ctrlb().modify(|_, w| w.freerun().set_bit());
        while self.adc.syncbusy().read().ctrlb().bit_is_set() {}
    }

    /// Enables an interrupt when conversion is ready.
    #[inline]
    fn enable_interrupts(&mut self) {
        //self.adc.intflag().write(|w| w.resrdy().set_bit());
        self.adc.intenset().write(|w| w.resrdy().set_bit());
        self.sync();
    }

    /// Disables the interrupt for when conversion is ready.
    #[inline]
    fn disable_interrupts(&mut self) {
        self.adc.intenclr().write(|w| w.resrdy().set_bit());
        self.sync();
    }

    #[inline]
    fn result(&self) -> u16 {
        self.adc.result().read().result().bits()
    }

    #[inline]
    fn is_interrupt(&self) -> bool {
        self.adc.intflag().read().bits() != 0
    }

    #[inline]
    fn clear_interrupt(&self) {
        self.adc.intflag().write(|w| w.resrdy().set_bit());
    }

    #[inline]
    fn mux(&mut self, ch: u8) {
        self.adc
            .inputctrl()
            .modify(|_, w| unsafe { w.muxpos().bits(ch) });
        self.sync()
    }

    #[cfg(feature = "async")]
    #[inline]
    pub fn into_future<F>(self, _irqs: F) -> Adc<I, F>
    where
        F: crate::async_hal::interrupts::Binding<I::Interrupt, async_api::InterruptHandler<I>>,
    {
        use crate::async_hal::interrupts::InterruptSource;
        unsafe {
            I::Interrupt::unpend();
            I::Interrupt::enable();
        }
        Adc {
            adc: self.adc,
            _irqs: PhantomData,
        }
    }
}

#[cfg(feature = "async")]
impl<I: AdcInstance, F> Adc<I, F>
where
    F: crate::async_hal::interrupts::Binding<I::Interrupt, async_api::InterruptHandler<I>>,
{
    #[inline]
    pub async fn read(&mut self, ch: u8) -> u16 {
        use core::{future::poll_fn, task::Poll};
        self.disable_interrupts();
        self.mux(ch);
        self.power_up(); // Enable ADC
        self.start_conversion();
        let result = poll_fn(|cx| {
            if self.is_interrupt() {
                self.clear_interrupt();
                self.disable_interrupts();
                return Poll::Ready(self.result());
            }
            I::waker().register(cx.waker());
            self.enable_interrupts();

            if self.is_interrupt() {
                self.clear_interrupt();
                self.disable_interrupts();
                return Poll::Ready(self.result());
            }
            Poll::Pending
        })
        .await;
        self.power_down();
        result
    }

    /*
    pub async fn read_into_buffer<X: Fn(usize)>(&mut self, ch: u8, buffer: &mut [u16], f: X) {
        if buffer.is_empty() {
            return;
        }
        use core::{future::poll_fn, task::Poll};
        self.disable_interrupts();
        self.sync();
        self.mux(ch);
        self.sync();
        self.adc.ctrla().modify(|_, w| w.enable().set_bit()); // Enable ADC
        self.sync();
        self.adc.swtrig().modify(|_, w| w.start().set_bit());
        let mut pos = 0;
        poll_fn(|cx| {
            if self.is_interrupt() {
                self.clear_interrupt();
                buffer[pos] = self.result();
                pos += 1;
                if pos == buffer.len() {
                    self.disable_interrupts();
                    return Poll::Ready(());
                }
            }
            f(pos);
            I::waker().register(cx.waker());
            self.enable_interrupts();
            if self.is_interrupt() {
                self.clear_interrupt();
                buffer[pos] = self.result();
                pos += 1;
                if pos == buffer.len() {
                    self.disable_interrupts();
                    return Poll::Ready(());
                }
            }
            Poll::Pending
        })
        .await;
        self.adc.ctrla().modify(|_, w| w.enable().clear_bit()); // Stop ADC (No sync required)
    }
    */
}

// Here I declare the number of channels for the SAME51 ADC
// TODO: declare channels for other chip variants
#[hal_cfg(any("adc-d5x"))]
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
                #[inline]
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
