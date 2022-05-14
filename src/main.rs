#![no_std]
#![no_main]

// Rust Embedded stuff
use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;

// Defmt is a microcontroller oriented formatting library
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use rp_pico::hal;

use hal::clocks::Clock;
use hal::gpio;
use hal::pac::interrupt;

// USB Device support
use hal::usb::UsbBus;
use usb_device::{class_prelude::*, prelude::*};
// HID
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MediaKeyboardReport;
use usbd_hid::hid_class::HIDClass;

use rotary_encoder_embedded::{Direction, RotaryEncoder};

// Global USB objects
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static USB_HID: Mutex<RefCell<Option<HIDClass<hal::usb::UsbBus>>>> = Mutex::new(RefCell::new(None));

type DTPin = gpio::Pin<gpio::bank0::Gpio3, gpio::PullUpInput>;
type CLKPin = gpio::Pin<gpio::bank0::Gpio4, gpio::PullUpInput>;
type SwitchPin = gpio::Pin<gpio::bank0::Gpio2, gpio::PullUpInput>;

type RotaryEncoderContext = (RotaryEncoder<DTPin, CLKPin>, SwitchPin);

static GLOBAL_ROTARY_ENCODER_CONTEXT: Mutex<RefCell<Option<RotaryEncoderContext>>> =
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
    let usb_hid = HIDClass::new(usb_bus_reference, MediaKeyboardReport::desc(), 100); // Very low polling interval
                                                                                      // Give away rotary encoder object
    cortex_m::interrupt::free(|cs| {
        USB_HID.borrow(cs).replace(Some(usb_hid));
    });

    let usb_device = UsbDeviceBuilder::new(usb_bus_reference, UsbVidPid(0x1337, 0xb00b))
        .manufacturer("Pini")
        .product("Grigioer")
        .serial_number("1234")
        .device_class(0xef) // from: https://www.usb.org/defined-class-codes
        .build();
    unsafe {
        // Set the global USB_BUS object
        USB_DEVICE = Some(usb_device);
    }

    // Set up the pins and make sure they interrupt on both edges.
    let rotary_dt = pins.gpio3.into_mode();
    rotary_dt.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    rotary_dt.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);

    let rotary_clk = pins.gpio4.into_mode();
    rotary_clk.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    rotary_clk.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);

    let switch_pin = pins.gpio2.into_mode();
    switch_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

    // Initialize the rotary encoder
    let rotary_encoder = RotaryEncoder::new(rotary_dt, rotary_clk);
    // Give away rotary encoder object
    cortex_m::interrupt::free(|cs| {
        GLOBAL_ROTARY_ENCODER_CONTEXT
            .borrow(cs)
            .replace(Some((rotary_encoder, switch_pin)));
    });

    //
    // Interrupt unmasking and Main loop
    //
	info!("sup");
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
        //send_media_keyboard_report(MediaKeyboardReport { usage_id: 0xe2 });
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // Claim the rotary encoder context first time this interrupt runs.
    // It will not and cannot be used in the MCU thread any more.
    static mut ROTARY_ENCODER_CONTEXT: Option<RotaryEncoderContext> = None;

    if ROTARY_ENCODER_CONTEXT.is_none() {
        cortex_m::interrupt::free(|cs| {
            *ROTARY_ENCODER_CONTEXT = GLOBAL_ROTARY_ENCODER_CONTEXT.borrow(cs).take();
        });
    }

    if let Some((rotary_encoder, switch_pin)) = ROTARY_ENCODER_CONTEXT {
        if switch_pin.interrupt_status(gpio::Interrupt::EdgeLow) {
            // A switch press
            debug!("BANK0 interrupt from switch pin.");
            // Read from the pins and then clear the interrupt
            if switch_pin.is_low().unwrap() {
                send_media_keyboard_report(MediaKeyboardReport { usage_id: 0xe2 });
            }
            switch_pin.clear_interrupt(gpio::Interrupt::EdgeLow);
        } else {
            // A rotation
            rotary_encoder.update();
            // TODO - concice logic for determining the origin of the
            // interrupt - since it can be both edges on both pins.
            // At the moment, we just clear all interrupts which is
            // fine - its unlikely we'll be missing consecutive edges.
            let pins = rotary_encoder.borrow_pins();
            pins.0.clear_interrupt(gpio::Interrupt::EdgeHigh);
            pins.0.clear_interrupt(gpio::Interrupt::EdgeLow);

            pins.1.clear_interrupt(gpio::Interrupt::EdgeHigh);
            pins.1.clear_interrupt(gpio::Interrupt::EdgeLow);

            match rotary_encoder.direction() {
                Direction::Clockwise => {
                    send_media_keyboard_report(MediaKeyboardReport { usage_id: 0xE9 });
                }
                Direction::Anticlockwise => {
                    send_media_keyboard_report(MediaKeyboardReport { usage_id: 0xEA });
                }
                Direction::None => {}
            }
        }
    }
    cortex_m::asm::sev();
}

fn send_media_keyboard_report(report: MediaKeyboardReport) -> Result<usize, usb_device::UsbError> {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut usb_hid) = USB_HID.borrow(cs).borrow_mut().deref_mut() {
            return usb_hid.push_input(&report);
        }
		Ok(0)
    })
}

// USB Interrupt handler
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_device = USB_DEVICE.as_mut().unwrap();
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut usb_hid) = USB_HID.borrow(cs).borrow_mut().deref_mut() {
            usb_device.poll(&mut [usb_hid]);
        }
    });
}
