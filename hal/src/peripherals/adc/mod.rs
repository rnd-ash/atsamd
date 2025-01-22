use core::{marker::PhantomData, ops::Deref};

use atsamd_hal_macros::{hal_cfg, hal_module};
use pac::dmac::channel::chctrla::Trigactselect;
use pac::{Mclk, Peripherals};
use seq_macro::seq;

use crate::{
    dmac::{self, sram::DmacDescriptor, AnyChannel, Beat, ReadyFuture},
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

#[derive(Debug)]
pub enum Error {
    /// Clock too fast.
    ///
    /// The ADC requires that it's fed a GCLK running at MAXIMUM 100 MHz
    /// (SAMDx5x). Above 100°C, the GCLK must run at or below 90 MHz.
    ClockTooFast,
    /// Buffer overflowed
    BufferOverrun,
    /// Temperature sensor not enabled
    ///
    /// This is returned when attempting to read the CPU temperature, and
    /// the SUPC peripheral has not been configured correctly to expose
    /// the temperature sensors.
    TemperatureSensorNotEnabled,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum CpuVoltageSource {
    /// Core voltage
    Core,
    /// VBAT supply voltage
    Vbat,
    /// IO supply voltage
    Io,
}

bitflags::bitflags! {
    #[derive(Clone, Copy)]
    pub struct Flags: u8 {
        /// Window monitor interrupt
        const WINMON = 0x04;
        /// Buffer overrun interrupt
        const OVERRUN = 0x02;
        /// Result ready interrupt
        const RESRDY = 0x01;
    }
}

/// Marker for which ADC has access to the CPUs internal sensors
pub trait PrimaryAdc {}

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

// TODO: The next few lines will need to be adjusted for SAMD11 and SAMD21: they
// only have 1 ADC
pub struct Adc0 {
    _adc: pac::Adc0,
}

impl PrimaryAdc for Adc0 {}

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
    _adc: pac::Adc1,
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
/// [`AdcPin`] in order to perform conversions. By default, channels don't hold
/// any pin when they are created by [`Adc::new`]. Use
/// [`Channel::with_pin`](Self::with_pin) to give a pin to this [`Channel`].
pub struct Channel<I: AdcInstance, Id: ChId, P> {
    _pin: P,
    _instance: PhantomData<I>,
    _id: PhantomData<Id>,
}

// These methods are only implemented for a Channel that doesn't hold a pin yet
impl<I: AdcInstance, Id: ChId> Channel<I, Id, NoneT> {
    // NOTE: `new`` must be private so a channel isn't accidentally created outside
    // this module, breaking the typelevel guarantees laid out by the adc driver
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
        // pin to the alternate B function, since the with_pin method is the only way to
        // insert a pin into the Channel.
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
    pub fn read_blocking<F>(&self, adc: &mut Adc<I, F>) -> u16 {
        adc.read_blocking(Id::ID)
    }

    #[inline]
    pub fn read_buffer_blocking<F>(
        &self,
        adc: &mut Adc<I, F>,
        dst: &mut [u16],
    ) -> Result<(), Error> {
        adc.read_buffer_blocking(Id::ID, dst)
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
    pub async fn read_buffer<F>(&self, adc: &mut Adc<I, F>, dst: &mut [u16]) -> Result<(), Error>
    where
        F: crate::async_hal::interrupts::Binding<I::Interrupt, async_api::InterruptHandler<I>>,
    {
        adc.read_buffer(Id::ID, dst).await
    }
}

/// ADC Instance
pub struct Adc<I: AdcInstance, F = NoneT> {
    adc: I::Instance,
    _irqs: PhantomData<F>,
    cfg: AdcSettingsBuilder,
}

pub struct AdcFuture;

impl<I: AdcInstance> Adc<I, NoneT> {
    /// Construct a new ADC instance
    ///
    /// ## Important
    /// This function will return `Err` if the clock source provided
    /// is faster than 100Mhz, since this is the maximum frequency for GCLK_ADCx
    /// as per the datasheet.
    ///
    /// NOTE: If you plan to run the chip above 100°C, then the maximum GCLK
    /// frequency for the ADC is restricted to 90Mhz for stable performance.
    #[inline]
    pub fn new(
        adc: I::Instance,
        settings: AdcSettingsBuilder,
        mclk: &mut Mclk,
        clock: I::Clock,
    ) -> Result<(Self, Channels<I>), Error> {
        if (clock.into() as Hertz).to_Hz() > 100_000_000 {
            // Clock source is too fast
            return Err(Error::ClockTooFast);
        }
        I::enable_mclk(mclk);

        let mut new_adc = Self {
            adc,
            _irqs: PhantomData,
            cfg: settings.clone(),
        };
        new_adc.configure(settings);
        Ok((new_adc, Channels::new()))
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
            cfg: self.cfg,
            _irqs: PhantomData,
        }
    }
}

impl<I: AdcInstance, F> Adc<I, F> {
    #[inline]
    pub fn configure(&mut self, settings: AdcSettingsBuilder) {
        // Reset ADC here as we cannot guarantee its state
        // This also disables the ADC
        self.software_reset();
        I::calibrate(&self.adc);
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
        self.sync();
        self.adc
            .ctrlb()
            .modify(|_, w| w.ressel().variant(settings.bit_width));
        self.sync();

        self.adc
            .sampctrl()
            .modify(|_, w| unsafe { w.samplen().bits(settings.sample_clock_cycles) }); // sample length
        self.sync();
        self.adc.inputctrl().modify(|_, w| w.muxneg().gnd()); // No negative input (internal gnd)
        self.sync();

        match settings.accumulation {
            AdcAccumulation::Single => {
                // 1 sample to be used as is
                self.adc.avgctrl().modify(|_, w| {
                    w.samplenum().variant(Samplenumselect::_1);
                    unsafe { w.adjres().bits(0) }
                });
            }
            AdcAccumulation::Average(adc_sample_count) => {
                // A total of `adc_sample_count` elements will be averaged by the ADC
                // before it returns the result
                self.adc.avgctrl().modify(|_, w| {
                    w.samplenum().variant(adc_sample_count);
                    unsafe {
                        // Table 45-3 SAME51 datasheet
                        w.adjres()
                            .bits(core::cmp::min(adc_sample_count as u8, 0x04))
                    }
                });
            }
            AdcAccumulation::Summed(adc_sample_count) => {
                // A total of `adc_sample_count` elements will be summed by the ADC
                // before it returns the result
                self.adc.avgctrl().modify(|_, w| {
                    w.samplenum().variant(adc_sample_count);
                    unsafe { w.adjres().bits(0) }
                });
            }
        }

        self.sync();
        self.set_reference(settings.vref);
    }

    /// Converts our ADC Reading (0-n) to the range 0.0-1.0, where 1.0 = 2^(reading_bitwidth)
    fn reading_to_f32(&self, raw: u16) -> f32 {
        let max = match self.cfg.bit_width {
            AdcResolution::_16bit => 65536,
            AdcResolution::_12bit => 4096,
            AdcResolution::_10bit => 1024,
            AdcResolution::_8bit => 256,
        };
        raw as f32 / max as f32
    }

    #[inline]
    fn set_reference(&mut self, reference: Reference) {
        self.adc
            .refctrl()
            .modify(|_, w| w.refsel().variant(reference));
        self.sync();
    }

    #[inline]
    pub fn read_blocking(&mut self, ch: u8) -> u16 {
        // Clear overrun errors that might've occured before we try to read anything
        let _ = self.check_and_clear_flags(self.read_flags());
        self.disable_interrupts(Flags::all());
        self.mux(ch);
        self.power_up();
        self.start_conversion();
        self.clear_flags(Flags::RESRDY);
        let _discard = self.conversion_result();
        while !self.read_flags().contains(Flags::RESRDY) {
            core::hint::spin_loop();
        }
        self.power_down();
        let res = self.conversion_result();
        res
    }

    #[inline]
    pub fn read_buffer_blocking(&mut self, ch: u8, dst: &mut [u16]) -> Result<(), Error> {
        // Clear overrun errors that might've occured before we try to read anything
        let _ = self.check_and_clear_flags(self.read_flags());
        self.enable_freerunning();

        self.disable_interrupts(Flags::all());
        self.mux(ch);
        self.power_up();
        self.start_conversion();
        for result in dst.iter_mut() {
            while !self.read_flags().contains(Flags::RESRDY) {
                core::hint::spin_loop();
            }
            *result = self.conversion_result();
            self.check_and_clear_flags(self.read_flags())?;
        }
        self.power_down();
        self.disable_freerunning();
        Ok(())
    }

    /// Return the underlying ADC PAC object
    ///
    /// You must also return all channels to the ADC to free its resources.
    #[inline]
    pub fn free(mut self, _channels: Channels<I>) -> I::Instance {
        self.software_reset();
        self.adc
    }

    /// Reset the peripheral.
    ///
    /// This also disables the ADC.
    #[inline]
    fn software_reset(&mut self) {
        self.adc.ctrla().modify(|_, w| w.swrst().set_bit());
        self.sync();
    }
}

impl<I: AdcInstance + PrimaryAdc, F> Adc<I, F> {
    #[inline]
    fn tp_tc_to_temp(&self, tp: f32, tc: f32) -> f32 {
        let tl = calibration::tl();
        let th = calibration::th();
        let vpl = calibration::vpl() as f32;
        let vph = calibration::vph() as f32;
        let vcl = calibration::vcl() as f32;
        let vch = calibration::vch() as f32;

        (tl * vph * tc - vpl * th * tc - tl * vch * tp + th * vcl * tp)
            / (vcl * tp - vch * tp - vpl * tc + vph * tc)
    }

    #[inline]
    /// Returns the CPU temperature in degrees C
    ///
    /// This requires that the [pac::Supc] peripheral is configured with
    /// tsen and ondemand bits enabled, otherwise this function will return
    /// [Error::TemperatureSensorNotEnabled]
    pub fn read_cpu_temperature_blocking(&mut self, supc: &pac::Supc) -> Result<f32, Error> {
        let vref = supc.vref().read();
        if vref.tsen().bit_is_clear() || vref.ondemand().bit_is_clear() {
            return Err(Error::TemperatureSensorNotEnabled);
        }
        let mut tp = self.read_blocking(0x1C) as f32;
        let mut tc = self.read_blocking(0x1D) as f32;

        if let AdcAccumulation::Summed(sum) = self.cfg.accumulation {
            // to prevent incorrect readings, divide by number of samples if the
            // ADC was already configured in summation mode
            let div: f32 = (2u16.pow(sum as u32)) as f32;
            tp /= div;
            tc /= div;
        }
        Ok(self.tp_tc_to_temp(tp, tc))
    }

    /// Read one of the CPU internal voltage supply, and return the value in
    /// millivolts (Volts/1000)
    pub fn read_internal_voltage_blocking(&mut self, src: CpuVoltageSource) -> u16 {
        let chan = match src {
            CpuVoltageSource::Core => 0x18,
            CpuVoltageSource::Vbat => 0x19,
            CpuVoltageSource::Io => 0x1A,
        };
        // Before reading, we have to select VDDANA as our reference voltage
        // so we get the full 3v3 range
        if self.cfg.vref != Reference::Intvcc1 {
            // Modify it
            self.set_reference(Reference::Intvcc1);
        }

        let mut adc_val = self.read_blocking(chan);
        if let AdcAccumulation::Summed(sum) = self.cfg.accumulation {
            let div: u16 = 2u16.pow(sum as u32);
            adc_val /= div;
        }
        let mut res = self.reading_to_f32(adc_val) * 3.3 * 4.0;

        // Restore our settings
        if Reference::Intvcc1 != self.cfg.vref {
            self.set_reference(self.cfg.vref);
        }
        (res * 1000.0) as u16
    }
}

impl<I: AdcInstance, T> Adc<I, T> {
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
            cfg: self.cfg,
        }
    }

    #[inline]
    fn read_flags(&self) -> Flags {
        let bits = self.adc.intflag().read().bits();
        Flags::from_bits_truncate(bits)
    }

    #[inline]
    fn clear_flags(&mut self, flags: Flags) {
        unsafe {
            self.adc.intflag().write(|w| w.bits(flags.bits()));
        }
    }

    /// Check the interrupt flags, clears them and returns `Err` if an overflow
    /// occured
    #[inline]
    fn check_and_clear_flags(&mut self, flags: Flags) -> Result<(), Error> {
        // Keep a copy around so we can check for errors later
        let flags_to_clear = flags;
        self.clear_flags(flags_to_clear);

        if flags.contains(Flags::OVERRUN) {
            Err(Error::BufferOverrun)
        } else {
            Ok(())
        }
    }

    #[inline]
    fn sync(&self) {
        // Slightly more performant than checking the individual bits
        // since we avoid an extra instruction to bit shift
        while self.adc.syncbusy().read().bits() != 0 {
            core::hint::spin_loop();
        }
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
        // The double trigger here is in case the VREF value changed between
        // reads, this discards the conversion made just after the VREF changed,
        // which the data sheet tells us to do in order to not get a faulty reading
        // right after changing VREF value
        self.adc.swtrig().modify(|_, w| w.start().set_bit());
        self.sync();
        self.adc.swtrig().modify(|_, w| w.start().set_bit());
    }

    #[inline]
    fn enable_freerunning(&mut self) {
        self.adc.ctrlb().modify(|_, w| w.freerun().set_bit());
        self.sync();
    }

    #[inline]
    fn disable_freerunning(&mut self) {
        self.adc.ctrlb().modify(|_, w| w.freerun().set_bit());
        self.sync();
    }

    #[inline]
    fn read_flags(&self) -> Flags {
        let bits = self.adc.intflag().read().bits();
        Flags::from_bits_truncate(bits)
    }

    #[inline]
    fn clear_flags(&mut self, flags: Flags) {
        unsafe {
            self.adc.intflag().write(|w| w.bits(flags.bits()));
        }
    }

    /// Check the interrupt flags, clears them and returns `Err` if an overflow
    /// occured
    #[inline]
    fn check_and_clear_flags(&mut self, flags: Flags) -> Result<(), Error> {
        // Keep a copy around so we can check for errors later
        let flags_to_clear = flags;
        self.clear_flags(flags_to_clear);

        if flags.contains(Flags::OVERRUN) {
            Err(Error::BufferOverrun)
        } else {
            Ok(())
        }
    }

    /// Enables an interrupt when conversion is ready.
    #[inline]
    #[allow(dead_code)]
    fn enable_interrupts(&mut self, flags: Flags) {
        unsafe { self.adc.intenset().write(|w| w.bits(flags.bits())) };
    }

    /// Disables the interrupt for when conversion is ready.
    #[inline]
    fn disable_interrupts(&mut self, flags: Flags) {
        unsafe { self.adc.intenclr().write(|w| w.bits(flags.bits())) };
    }

    #[inline]
    fn conversion_result(&self) -> u16 {
        self.adc.result().read().result().bits()
    }

    #[inline]
    fn mux(&mut self, ch: u8) {
        self.adc
            .inputctrl()
            .modify(|_, w| unsafe { w.muxpos().bits(ch) });
        self.sync()
    }
}

#[cfg(feature = "async")]
impl<I: AdcInstance, F> Adc<I, F>
where
    F: crate::async_hal::interrupts::Binding<I::Interrupt, async_api::InterruptHandler<I>>,
{
    #[inline]
    pub async fn read(&mut self, ch: u8) -> u16 {
        // Clear overrun errors that might've occured before we try to read anything
        self.mux(ch);
        self.power_up();
        let _ = self.check_and_clear_flags(self.read_flags());
        self.start_conversion();
        // Here we explicitly ignore the result, because we know that
        // overrun errors are impossible since the ADC is configured in one-shot mode.
        let _ = self.wait_flags(Flags::RESRDY).await;
        let result = self.conversion_result();
        self.power_down();
        result
    }

    #[inline]
    pub async fn read_buffer(&mut self, ch: u8, dst: &mut [u16]) -> Result<(), Error> {
        // Clear overrun errors that might've occured before we try to read anything
        self.enable_freerunning();

        self.mux(ch);
        self.power_up();
        let _ = self.check_and_clear_flags(self.read_flags());
        self.start_conversion();
        for result in dst.iter_mut() {
            self.wait_flags(Flags::RESRDY).await?;
            *result = self.conversion_result();
        }

        self.power_down();
        self.disable_freerunning();
        Ok(())
    }
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
