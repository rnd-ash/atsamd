use core::marker::PhantomData;

use crate::{
    adc::{Adc, AdcInstance, Error, Flags},
    async_hal::interrupts::Handler,
};

use embassy_sync::waitqueue::AtomicWaker;

#[allow(clippy::declare_interior_mutable_const)]
const NEW_WAKER: AtomicWaker = AtomicWaker::new();
pub static ADC_WAKERS: [AtomicWaker; 2] = [NEW_WAKER; 2];

/// Interrupt handler for the ADC peripheral.
pub struct InterruptHandler<A: AdcInstance> {
    _private: (),
    _adc: PhantomData<A>,
}

impl<A: AdcInstance> crate::typelevel::Sealed for InterruptHandler<A> {}

impl<A: AdcInstance> Handler<A::Interrupt> for InterruptHandler<A> {
    unsafe fn on_interrupt() {
        let mut peripherals = crate::pac::Peripherals::steal();
        let adc = A::peripheral_reg_block(&mut peripherals);

        let flags_pending = Flags::from_bits_truncate(adc.intflag().read().bits());
        let enabled_flags = Flags::from_bits_truncate(adc.intenset().read().bits());

        if enabled_flags.intersects(flags_pending) {
            adc.intenclr().write(|w| w.bits(flags_pending.bits()));
            // Wake up!
            A::waker().wake();
        }
    }
}

impl<I: AdcInstance, F> Adc<I, F>
where
    F: crate::async_hal::interrupts::Binding<I::Interrupt, InterruptHandler<I>>,
{
    #[inline]
    pub(super) async fn wait_flags(&mut self, flags_to_wait: Flags) -> Result<(), Error> {
        use core::task::Poll;

        // We automatically check for errors
        let flags_to_wait = flags_to_wait | Flags::OVERRUN;
        self.disable_interrupts(Flags::all());

        core::future::poll_fn(|cx| {
            // Scope maybe_pending so we don't forget to re-poll the register later down.
            {
                let maybe_pending = self.read_flags();
                if flags_to_wait.intersects(maybe_pending) {
                    let result = self.check_and_clear_flags(maybe_pending);
                    self.disable_interrupts(flags_to_wait);
                    return Poll::Ready(result);
                }
            }

            I::waker().register(cx.waker());
            self.enable_interrupts(flags_to_wait);

            let maybe_pending = self.read_flags();

            if !flags_to_wait.intersects(maybe_pending) {
                Poll::Pending
            } else {
                let result = self.check_and_clear_flags(maybe_pending);
                self.disable_interrupts(flags_to_wait);
                Poll::Ready(result)
            }
        })
        .await
    }
}
