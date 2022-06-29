use rp2040_hal as hal;

pub type SwitchPin = hal::gpio::Pin<hal::gpio::bank0::Gpio20, hal::gpio::PullUpInput>;
pub type DTPin = hal::gpio::Pin<hal::gpio::bank0::Gpio18, hal::gpio::PullUpInput>;
pub type CLKPin = hal::gpio::Pin<hal::gpio::bank0::Gpio19, hal::gpio::PullUpInput>;

// Use PAC from rp_pico module - for some reason I can't get rid of this depedency
// Including the original rp2040_pac manually does not seem to work.
pub use rp_pico::hal::pac;

hal::bsp_pins!(
    Gpio20 {
        name: switch,
        aliases: {
            PullUpInput: EncoderSwitchPin
        }
    },
    Gpio19 {
        name: clk,
        aliases: {
            PullUpInput: EncoderCLKPin
        }
    },
    Gpio18 {
        name: dt,
        aliases: {
            PullUpInput: EncoderDTPin
        }
    },

);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
