#![no_std]
#![no_main]

// Rust Embedded stuff
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;

// Defmt is a microcontroller oriented formatting library
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use rp_pico::hal;

use hal::clocks::Clock;
use hal::pac::interrupt;

// USB Device support
use hal::usb::UsbBus;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use rotary_encoder_embedded::{Direction, RotaryEncoder};
use rp2040_hal::gpio::Interrupt::EdgeHigh;
use rp2040_hal::gpio::Interrupt::EdgeLow;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
// Global USB objects
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

use rp2040_hal::gpio;
type DTPin = gpio::Pin<gpio::bank0::Gpio0, gpio::PullUpInput>;
type CLKPin = gpio::Pin<gpio::bank0::Gpio1, gpio::PullUpInput>;

static GLOBAL_ROTARY_ENCODER: Mutex<RefCell<Option<RotaryEncoder<DTPin, CLKPin>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    //
    // Setup
    //
    let mut pac = hal::pac::Peripherals::take().unwrap(); // Todo - find a better pattern than unwrap, very unrustlike
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::sio::Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let xtal_freq_hz = rp_pico::XOSC_CRYSTAL_FREQ;

    let clocks = hal::clocks::init_clocks_and_plls(
        xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    //
    // Auxiliary objects
    //
    let core = hal::pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Pin setup
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.led.into_push_pull_output();

    //
    // USB setup, bus, device, and driver

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Set the global USB_BUS object
        USB_BUS = Some(usb_bus);
    }
    let usb_bus_reference = unsafe { USB_BUS.as_ref().unwrap() }; // Not quite sure why this is needed, perhaps the old ref is moved?

    // Set up the driver
    let usb_serial = SerialPort::new(usb_bus_reference);
    unsafe {
        USB_SERIAL = Some(usb_serial);
    }

    let usb_device = UsbDeviceBuilder::new(usb_bus_reference, UsbVidPid(0x1337, 0xb00b))
        .manufacturer("Pini")
        .product("Grigioer")
        .serial_number("1234")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    unsafe {
        // Set the global USB_BUS object
        USB_DEVICE = Some(usb_device);
    }

    //
    // Main loop
    //
    // We should unmask the relevant interrupts now, and avoid sync issues.

    let rotary_dt = pins.gpio0.into_pull_up_input();
    let rotary_clk = pins.gpio1.into_pull_up_input();
    rotary_dt.set_interrupt_enabled(EdgeLow, true);
    rotary_dt.set_interrupt_enabled(EdgeHigh, true);
    rotary_clk.set_interrupt_enabled(EdgeLow, true);
    rotary_clk.set_interrupt_enabled(EdgeHigh, true);
    // Initialize the rotary encoder
    let mut rotary_encoder = RotaryEncoder::new(rotary_dt, rotary_clk);
    // Give away rotary encoder object
    cortex_m::interrupt::free(|cs| {
        GLOBAL_ROTARY_ENCODER
            .borrow(cs)
            .replace(Some(rotary_encoder));
    });

    debug!("unmasking interrupts.");
    // USB
    unsafe {
        hal::pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        hal::pac::NVIC::unmask(hal::pac::Interrupt::IO_IRQ_BANK0);
    };

    debug!("main loop starting.");
    loop {
        cortex_m::asm::wfe();
        // Light the LED to indicate we saw an interrupt.
        led_pin.set_high().unwrap();
        delay.delay_ms(100);
        led_pin.set_low().unwrap();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut ROTARY_ENCODER: Option<RotaryEncoder<DTPin, CLKPin>> = None;

    if ROTARY_ENCODER.is_none() {
        cortex_m::interrupt::free(|cs| {
            *ROTARY_ENCODER = GLOBAL_ROTARY_ENCODER.borrow(cs).take();
        });
    }

    if let Some(rotary_encoder) = ROTARY_ENCODER {
        // Update the encoder, which will compute its direction
        rotary_encoder.update();
        match rotary_encoder.direction() {
            Direction::Clockwise => {
                cortex_m::interrupt::free(|_| unsafe {
                    USB_SERIAL.as_mut().unwrap().write(b"ueep\r\n")
                })
                .unwrap();
                // Increment some value
            }
            Direction::Anticlockwise => {
                cortex_m::interrupt::free(|_| unsafe {
                    USB_SERIAL.as_mut().unwrap().write(b"eedown\r\n")
                })
                .unwrap();
            }
            Direction::None => {
                // Do nothing
            }
        }
    }
    cortex_m::asm::sev();
}

// USB Interrupt handler
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_device = USB_DEVICE.as_mut().unwrap();
    let usb_serial = USB_SERIAL.as_mut().unwrap();
    usb_device.poll(&mut [usb_serial]);
    cortex_m::asm::sev();
}
