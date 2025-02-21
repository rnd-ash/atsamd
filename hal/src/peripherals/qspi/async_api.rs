use embassy_sync::waitqueue::AtomicWaker;

use super::Flags;
use crate::async_hal::interrupts::{Handler, QSPI};

#[allow(clippy::declare_interior_mutable_const)]

pub static QSPI_WAKER: AtomicWaker = AtomicWaker::new();

pub struct QspiInterruptHandler {}

impl crate::typelevel::Sealed for QspiInterruptHandler {}

impl Handler<QSPI> for QspiInterruptHandler {
    unsafe fn on_interrupt() {
        let peripherals = crate::pac::Peripherals::steal();
        let qspi = peripherals.qspi;
        let flags_pending = Flags::from_bits_truncate(qspi.intflag().read().bits());
        let enabled_flags = Flags::from_bits_truncate(qspi.intenset().read().bits());
        if enabled_flags.intersects(flags_pending) {
            qspi.intenclr().write(|w| w.bits(flags_pending.bits()));
            // Wake up!
            QSPI_WAKER.wake();
        }
    }
}
