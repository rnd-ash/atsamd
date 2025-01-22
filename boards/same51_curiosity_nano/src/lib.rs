#![no_std]
#![deny(missing_docs)]

//! Board support crate for Adafruit's Feather M4 Express,
//! an ATSAMD51-based board in Feather form factor.

#[cfg(feature = "rt")]
pub use cortex_m_rt::entry;

pub use atsamd_hal as hal;
pub use hal::ehal;
pub use hal::pac;

use hal::clock::GenericClockController;
use hal::sercom::{
    i2c, spi,
    uart::{self, BaudMode, Oversampling},
    IoSet1, UndocIoSet1,
};
use hal::time::Hertz;

#[cfg(feature = "usb")]
use hal::usb::usb_device::bus::UsbBusAllocator;
#[cfg(feature = "usb")]
pub use hal::usb::UsbBus;

hal::bsp_peripherals!(
    Sercom1 { SpiSercom }
    Sercom2 { I2cSercom }
    Sercom5 { UartSercom }
);

hal::bsp_pins!(
    PA02 {
        /// Analog pin 0.  Can act as a true analog output
        /// as it has a DAC (which is not currently supported
        /// by this hal) as well as input.
        name: a0,
    }
    PA05 {
        /// Analog Pin 1
        name: a1,
    }
    PB08 {
        /// Analog Pin 2
        name: a2,
    }
    PB09 {
        /// Analog Pin 3
        name: a3,
    }
    PA04 {
        /// Analog Pin 4
        name: a4,
    }
    PA06 {
        /// Analog Pin 5
        name: a5,
    }
    PB01 {
        /// Analog Vdiv (1/2 resistor divider for monitoring the battery)
        name: battery,
    }

    PB17 {
        /// Pin 0, UART rx
        name: d0,
        aliases: {
            AlternateC: UartRx
        }
    }
    PB16 {
        /// Pin 1, UART tx
        name: d1,
        aliases: {
            AlternateC: UartTx
        }
    }
    PA14 {
        /// Pin 4, PWM capable
        name: d4,
    }
    PA16 {
        /// Pin 5, PWM capable
        name: d5,
    }
    PA18 {
        /// Pin 6, PWM capable
        name: d6,
    }
    PB03 {
        /// Neopixel Pin
        name: neopixel,
    }
    PA19 {
        /// Pin 9, PWM capable.  Also analog input (A7)
        name: d9,
    }
    PA20 {
        /// Pin 10, PWM capable
        name: d10,
    }
    PA21 {
        /// Pin 11, PWM capable
        name: d11,
    }
    PA22 {
        /// Pin 12, PWM capable
        name: d12,
    }
    PA23 {
        /// Pin 13, which is also attached to the red LED. PWM capable.
        name: d13,
        aliases: {
            PushPullOutput: RedLed,
            AlternateE: RedLedPwm
        }
    }
    PA12 {
        /// The I2C data line
        name: sda,
        aliases: {
            AlternateC: Sda
        }
    }
    PA13 {
        /// The I2C clock line
        name: scl,
        aliases: {
            AlternateC: Scl
        }
    }
    PA17 {
        /// The SPI SCK
        name: sck,
        aliases: {
            AlternateC: Sclk
        }
    }
    PB23 {
        /// The SPI MOSI
        name: mosi,
        aliases: {
            AlternateC: Mosi
        }
    }
    PB22 {
        /// The SPI MISO
        name: miso,
        aliases: {
            AlternateC: Miso
        }
    }
    PA24 {
        /// The USB D- pad
        name: usb_dm,
        aliases: {
            AlternateH: UsbDm
        }
    }
    PA25 {
        /// The USB D+ pad
        name: usb_dp,
        aliases: {
            AlternateH: UsbDp
        }
    }
);

/// UART pads for the labelled RX & TX pins
pub type UartPads = uart::Pads<UartSercom, IoSet1, UartRx, UartTx>;

/// UART device for the labelled RX & TX pins
pub type Uart = uart::Uart<uart::Config<UartPads>, uart::Duplex>;

/// Convenience for setting up the labelled RX, TX pins to
/// operate as a UART device running at the specified baud.
pub fn uart(
    clocks: &mut GenericClockController,
    baud: impl Into<Hertz>,
    sercom: UartSercom,
    mclk: &mut pac::Mclk,
    rx: impl Into<UartRx>,
    tx: impl Into<UartTx>,
) -> Uart {
    let gclk0 = clocks.gclk0();

    let clock = &clocks.sercom5_core(&gclk0).unwrap();
    let baud = baud.into();
    let pads = uart::Pads::default().rx(rx.into()).tx(tx.into());
    uart::Config::new(mclk, sercom, pads, clock.freq())
        .baud(baud, BaudMode::Fractional(Oversampling::Bits16))
        .enable()
}
