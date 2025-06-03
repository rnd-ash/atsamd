use atsamd_hal_macros::hal_module;

#[hal_module(
    any("clock-d11", "clock-d21") => "d11.rs",
    "clock-d5x" => "d5x.rs",
)]
mod impls {}

pub use impls::*;
