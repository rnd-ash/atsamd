use core::{marker::PhantomData, ops::Deref};

use atsamd_hal_macros::{hal_cfg, hal_module};
use pac::Peripherals;
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

pub use impls::*;

#[cfg(feature = "async")]
mod async_api;
#[cfg(feature = "async")]
pub use async_api::*;

mod config;
pub use config::*;

#[hal_cfg(any("adc-d11", "adc-d21"))]
use crate::pac::adc as adc0;
#[hal_cfg(any("adc-d5x"))]
use crate::pac::adc0;

pub use adc0::avgctrl::Samplenumselect;
/// Samples per reading
pub use adc0::avgctrl::Samplenumselect as SampleRate;
/// Reading resolution in bits
pub use adc0::ctrlb::Resselselect as Resolution;
/// Reference voltage (or its source)
pub use adc0::refctrl::Refselselect as Reference;

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Clock too fast.
    ///
    /// The ADC requires that it's fed a GCLK that does not exceed a certain frequency.
    /// These maximums are:
    ///
    /// * **SAMD/E5x** - 100Mhz
    /// * **SAMC/D21** - 48Mhz
    /// * **SAMD11** - 48Mhz
    ///
    /// SAMx51 specific: If you are running the CPU at temperatures past 100C, then
    /// the maximum GCLK clock speed should be 90Mhz
    ClockTooFast,
    /// Buffer overflowed
    BufferOverrun,
    /// Temperature sensor not enabled
    ///
    /// This is returned when attempting to read the CPU temperature, and
    /// the SUPC peripheral has not been configured correctly to expose
    /// the temperature sensors.
    TemperatureSensorNotEnabled,
    /// Invalid ADC Setting sample bit width
    ///
    /// This can happen if you are trying to average/sum ADC values,
    /// and did not select 16bit bitwidth for ADC outputs
    InvalidSampleBitWidth,
}

#[hal_cfg(any("adc-d5x"))]
#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum CpuVoltageSource {
    /// Core voltage
    Core = 0x18,
    /// VBAT supply voltage
    Vbat = 0x19,
    /// IO supply voltage
    Io = 0x1A,
}

#[hal_cfg(any("adc-d21", "adc-d11"))]
#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum CpuVoltageSource {
    /// Core voltage
    Core = 0x1A,
    /// VBAT supply voltage
    Vbat = 0x1B,
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
    type Instance: Deref<Target = adc0::RegisterBlock>;
    type Clock: Into<Hertz>;

    fn peripheral_reg_block(p: &mut Peripherals) -> &adc0::RegisterBlock;

    #[hal_cfg(any("adc-d5x"))]
    fn enable_mclk(mclk: &mut pac::Mclk);

    #[hal_cfg(any("adc-d11", "adc-d21"))]
    fn enable_pm(pm: &mut pac::Pm);

    fn calibrate(instance: &Self::Instance);

    #[cfg(feature = "async")]
    fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
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
    cfg: Config,
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
    #[hal_cfg(any("adc-d5x"))]
    #[inline]
    pub fn new(
        adc: I::Instance,
        config: Config,
        mclk: &mut pac::Mclk,
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
            cfg: config.clone(),
        };
        new_adc.configure(config)?;
        Ok((new_adc, Channels::new()))
    }

    #[hal_cfg(any("adc-d11", "adc-d21"))]
    #[inline]
    pub fn new(
        adc: I::Instance,
        config: Config,
        pm: &mut pac::Pm,
        clock: I::Clock,
    ) -> Result<(Self, Channels<I>), Error> {
        if (clock.into() as Hertz).to_Hz() > 48_000_000 {
            // Clock source is too fast
            return Err(Error::ClockTooFast);
        }

        I::enable_pm(pm);
        let mut new_adc = Self {
            adc,
            _irqs: PhantomData,
            cfg: config.clone(),
        };
        new_adc.configure(config)?;
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
    /// Converts our ADC Reading (0-n) to the range 0.0-1.0, where
    /// 1.0 = 2^(reading_bitwidth)
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
    /// Read one of the CPU internal voltage supply, and return the value in
    /// millivolts (Volts/1000)
    pub fn read_internal_voltage_blocking(&mut self, src: CpuVoltageSource) -> u16 {
        let chan = src as u8;
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
        let res = self.reading_to_f32(adc_val) * 3.3 * 4.0;

        // Restore our settings
        if Reference::Intvcc1 != self.cfg.vref {
            self.set_reference(self.cfg.vref);
        }
        (res * 1000.0) as u16
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

// Channel implementation

#[hal_cfg(any("adc-d5x"))]
macro_rules! with_num_channels {
    ($some_macro:ident) => {
        $some_macro! {16}
    };
}

#[hal_cfg(any("adc-d21"))]
macro_rules! with_num_channels {
    ($some_macro:ident) => {
        $some_macro! {20}
    };
}

#[hal_cfg(any("adc-d11"))]
macro_rules! with_num_channels {
    ($some_macro:ident) => {
        $some_macro! {10}
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
