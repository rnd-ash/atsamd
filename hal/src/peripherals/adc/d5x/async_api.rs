use core::marker::PhantomData;

use crate::{adc::AdcInstance, async_hal::interrupts::Handler};

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
        let mut peripherals = unsafe { crate::pac::Peripherals::steal() };
        let adc = A::peripheral_reg_block(&mut peripherals);
        critical_section::with(|_| {
            // Just check if result ready is set. Todo - Handle overrun and other interrupt reasons
            if adc.intflag().read().resrdy().bit_is_set() {
                // Wake up!
                A::waker().wake();
            } else {
                // Handle other cases
            }
        })
    }
}
