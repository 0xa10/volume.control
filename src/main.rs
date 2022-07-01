#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use defmt_rtt as _;
#[cfg(not(debug_assertions))]
use panic_halt as _;
#[cfg(debug_assertions)]
use panic_probe as _;

mod boards;
mod hid;

#[rtic::app(device = board::pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    use core::mem::MaybeUninit;
    use defmt::{debug, info};
    use embedded_hal::digital::v2::InputPin;
    use embedded_hal::prelude::*;
    use embedded_time::duration::Extensions;

    use crate::boards::rev_ii as board;
    use rp2040_hal as hal; // Set the target board here

    use rp2040_monotonic::fugit::ExtU64;
    use rp2040_monotonic::*;

    use hal::usb::UsbBus;
    use usb_device::{class_prelude::*, prelude::*};

    use crate::hid::{VolumeControlInterface, VolumeControlReport, HID_REPORTING_INTERVAL};
    use usbd_human_interface_device::hid_class::prelude::*;

    use frunk::HList;

    use rotary_encoder_hal::{Direction, Rotary};

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Monotonic = Rp2040Monotonic;

    const WATCHDOG_INTERVAL_MICROS: u32 = HID_REPORTING_INTERVAL;

    #[shared]
    struct Shared {
        usb_hid: UsbHidClass<UsbBus, HList!(VolumeControlInterface<'static, UsbBus>,)>,
        usb_device: UsbDevice<'static, UsbBus>,
        switch_pin: board::SwitchPin,
        rotary_encoder: Rotary<board::DTPin, board::CLKPin>,
    }

    #[local]
    struct Local {
        watchdog: hal::watchdog::Watchdog,
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
            board::XOSC_CRYSTAL_FREQ,
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
        let pins = board::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // Rotary encoder GPIO setup
        let rotary_dt = pins.dt.into_mode();

        let rotary_clk = pins.clk.into_mode();
        let rotary_encoder = Rotary::new(rotary_dt, rotary_clk);

        // Switch GPIO setup
        let switch_pin = pins.switch.into_mode();
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

        #[cfg(feature = "watchdog")]
        {
            info!("Starting watchdog.");
            watchdog.start((WATCHDOG_INTERVAL_MICROS).microseconds());
            feed_watchdog::spawn().ok();
        }

        info!("Init complete, starting.");
        (
            Shared {
                usb_hid,
                usb_device,
                switch_pin,
                rotary_encoder,
            },
            Local { watchdog },
            init::Monotonics(Rp2040Monotonic::new(cx.device.TIMER)),
        )
    }

    fn reset_to_bootsel() -> ! {
        if cfg!(debug_assertions) {
            rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
        } else {
            // In release versions - use only PICOBOOT interface and not mass storage.
            rp2040_hal::rom_data::reset_to_usb_boot(0, 1);
        }
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
        let mut _mute_debounce_delay_ms: u32 = 0;
        let report =
            (cx.shared.rotary_encoder, cx.shared.switch_pin).lock(|rotary_encoder, switch_pin| {
                // Assemble volume control report
                let switch_pin_state = switch_pin.is_low().unwrap_or(false);
                #[cfg(feature = "safe-muting")]
                {
                    // Optional feature - delay HID task by 0.1 secs after muting/unmuting, to prevent bouncing.
                    if switch_pin_state {
                        _mute_debounce_delay_ms = 100;
                    }
                }
                let direction = rotary_encoder.update().unwrap_or(Direction::None);

                VolumeControlReport {
                    mute: switch_pin_state,
                    volume_increment: direction == Direction::Clockwise,
                    volume_decrement: direction == Direction::CounterClockwise,
                }
            });

        cx.shared
            .usb_hid
            .lock(|usb_hid| usb_hid.interface().write_report(&report))
            .ok();
        usb_hid_task::spawn_after(ExtU64::millis(u64::from(
            _mute_debounce_delay_ms + HID_REPORTING_INTERVAL,
        )))
        .ok();
    }

    #[task(priority = 1, local = [watchdog])]
    fn feed_watchdog(cx: feed_watchdog::Context) {
        let watchdog = cx.local.watchdog;

        watchdog.feed();
        feed_watchdog::spawn_after(ExtU64::micros(u64::from(WATCHDOG_INTERVAL_MICROS / 2))).ok();
        // Watchdog will be triggered if we miss two feeds
    }
}
