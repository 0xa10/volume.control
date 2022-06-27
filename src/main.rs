#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

mod hid;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    use core::mem::MaybeUninit;
    use defmt::{debug, info};
    use embedded_hal::digital::v2::InputPin;
    use embedded_hal::digital::v2::OutputPin;
    use embedded_hal::digital::v2::ToggleableOutputPin;

    use rp2040_hal as hal;
    use rp2040_monotonic::*;

    use hal::usb::UsbBus;
    use usb_device::{class_prelude::*, prelude::*};

    use crate::hid::{VolumeControlInterface, VolumeControlReport};
    use usbd_human_interface_device::hid_class::prelude::*;

    use frunk::HList;

    use rotary_encoder_hal::{Direction, Rotary};

    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Monotonic = Rp2040Monotonic;
    type LedPin = hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>;
    type SwitchPin = hal::gpio::Pin<hal::gpio::bank0::Gpio2, hal::gpio::PullUpInput>;
    type DTPin = hal::gpio::Pin<hal::gpio::bank0::Gpio5, hal::gpio::PullUpInput>;
    type CLKPin = hal::gpio::Pin<hal::gpio::bank0::Gpio6, hal::gpio::PullUpInput>;

    #[shared]
    struct Shared {
        usb_hid: UsbHidClass<UsbBus, HList!(VolumeControlInterface<'static, UsbBus>,)>,
        usb_device: UsbDevice<'static, UsbBus>,
        switch_pin: SwitchPin,
        rotary_encoder: Rotary<DTPin, CLKPin>,
    }

    #[local]
    struct Local {
        led: LedPin,
    }

    #[init(local = [
			usb_bus: MaybeUninit<UsbBusAllocator<UsbBus>> = MaybeUninit::uninit(),
		])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        info!("Starting init");
        // General MCU initialization
        let mut resets = cx.device.RESETS;
        let mut watchdog = hal::watchdog::Watchdog::new(cx.device.WATCHDOG);

        let clocks = hal::clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            cx.device.XOSC,
            cx.device.CLOCKS,
            cx.device.PLL_SYS,
            cx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = hal::Sio::new(cx.device.SIO);
        let pins = rp_pico::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // LED setup - only relevant for Pico
        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        // Rotary encoder GPIO setup
        let rotary_dt: DTPin = pins.gpio5.into_mode();

        let rotary_clk: CLKPin = pins.gpio6.into_mode();
        let rotary_encoder = Rotary::new(rotary_dt, rotary_clk);

        // Switch GPIO setup
        let switch_pin = pins.gpio2.into_mode();
        // Check if button is pressed during setup - boot to BOOTSEL mode if so
        if let Ok(switch_pin_low_on_init) = switch_pin.is_low() {
            if switch_pin_low_on_init {
                reset_to_bootsel();
            }
        }

        // USB bus and device setup
        let usb_bus: &'static _ = cx.local.usb_bus.write(UsbBusAllocator::new(UsbBus::new(
            cx.device.USBCTRL_REGS,
            cx.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));

        let usb_hid = UsbHidClassBuilder::new()
            .add_interface(VolumeControlInterface::default_config())
            .build(usb_bus);

        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x4853))
            .manufacturer("pini.grigio")
            .product("volume.control")
            .max_packet_size_0(64)
            .max_power(500)
            .build();

        // HID task setup
        usb_hid_task::spawn().ok();

        // Init done
        (
            Shared {
                usb_hid,
                usb_device,
                switch_pin,
                rotary_encoder,
            },
            Local { led },
            init::Monotonics(Rp2040Monotonic::new(cx.device.TIMER)),
        )
    }

    fn reset_to_bootsel() -> ! {
        // Taken from jannic/rp2040-panic-usb-boot - not quite sure what
        // here is strictly necessary and what can be removed.
        cortex_m::interrupt::disable();
        let p = unsafe { rp2040_hal::pac::Peripherals::steal() };
        if !(p.XOSC.status.read().stable().bit()) {
            p.XOSC.startup.write(|w| unsafe {
                w.delay().bits((12_000 /*kHz*/ + 128) / 256)
            });
            p.XOSC.ctrl.write(|w| {
                w.freq_range()
                    .variant(rp2040_hal::pac::xosc::ctrl::FREQ_RANGE_A::_1_15MHZ)
                    .enable()
                    .variant(rp2040_hal::pac::xosc::ctrl::ENABLE_A::ENABLE)
            });
            while !(p.XOSC.status.read().stable().bit()) {}
        }

        rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
        loop {}
    }


    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_device, usb_hid])]
    fn on_usb(cx: on_usb::Context) {
        debug!("Entered USB IRQ");
        (cx.shared.usb_device, cx.shared.usb_hid).lock(|usb_device, usb_hid| {
            usb_device.poll(&mut [usb_hid]);
        });
    }

    #[task(shared = [usb_device, usb_hid, rotary_encoder, switch_pin])]
    fn usb_hid_task(mut cx: usb_hid_task::Context) {
        let report = (cx.shared.rotary_encoder, cx.shared.switch_pin).lock(
            |rotary_encoder, switch_pin| {
                // Assemble volume control report
                let switch_pin_state = switch_pin.is_low().unwrap_or(false);
                let direction = rotary_encoder.update().unwrap_or(Direction::None);

                VolumeControlReport {
                        mute: switch_pin_state,
                        volume_increment: direction == Direction::Clockwise,
                        volume_decrement: direction == Direction::CounterClockwise,
                }
            });
        
        cx.shared.usb_hid.lock(|usb_hid| usb_hid.interface().write_report(&report)).ok();
        usb_hid_task::spawn_after(8.millis()).ok(); // TODO - figure out millis type
    }

    #[task(priority = 1, local = [led])]
    fn toggle_led(cx: toggle_led::Context, blink: bool) {
        if blink {
            toggle_led::spawn_after(100.millis(), false).ok();
        }
        cx.local.led.toggle().ok();
    }
}
