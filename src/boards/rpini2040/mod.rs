pub extern crate rp2040_hal as hal;

pub type SwitchPin = hal::gpio::Pin<hal::gpio::bank0::Gpio2, hal::gpio::PullUpInput>;
pub type DTPin = hal::gpio::Pin<hal::gpio::bank0::Gpio5, hal::gpio::PullUpInput>;
pub type CLKPin = hal::gpio::Pin<hal::gpio::bank0::Gpio6, hal::gpio::PullUpInput>;

// Use PAC from rp_pico module - for some reason I can't get rid of this depedency
// Including the original rp2040_pac manually does not seem to work.
pub use rp_pico::hal::pac;

hal::bsp_pins!(
    Gpio2 {
        name: switch,
        aliases: { PullUpInput: EncoderSwitchPin }
    },
    Gpio6 {
        name: clk,
        aliases: { PullUpInput: EncoderCLKPin }
    },
    Gpio5 {
        name: dt,
        aliases: { PullUpInput: EncoderDTPin }
    },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
