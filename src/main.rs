#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

mod hid;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    use core::mem::MaybeUninit;
    use defmt::{debug, error, info, Debug2Format};
    use embedded_hal::digital::v2::InputPin;
    use embedded_hal::digital::v2::OutputPin;
    use embedded_hal::digital::v2::ToggleableOutputPin;

    use rp2040_monotonic::*;
    use rp_pico::hal;
    use rp_pico::XOSC_CRYSTAL_FREQ;

    use hal::usb::UsbBus;
    use usb_device::{class_prelude::*, prelude::*};
    
    use usbd_human_interface_device::hid_class::prelude::*;
    use crate::hid::{VolumeControlInterface, VolumeControl};
    
	use frunk::HList;

    use rotary_encoder_hal::{Direction, Rotary};

    use heapless::spsc::{Consumer as Consumer, Producer, Queue};

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
        hid_consumer: Consumer<'static, VolumeControl, 4>,
        hid_producer: Producer<'static, VolumeControl, 4>,
        switch_pin: SwitchPin,
        rotary_encoder: Rotary<DTPin, CLKPin>,
    }

    #[local]
    struct Local {
        led: LedPin,
    }

    #[init(local = [
			usb_bus: MaybeUninit<UsbBusAllocator<UsbBus>> = MaybeUninit::uninit(),
			hid_queue: Queue<VolumeControl, 4> = Queue::new(),
		])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        info!("Starting init");
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
        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        // Set up the pins and make sure they interrupt on both edges.
        let rotary_dt: DTPin = pins.gpio5.into_mode();
        rotary_dt.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);
        rotary_dt.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

        let rotary_clk: CLKPin = pins.gpio6.into_mode();
        rotary_clk.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);
        rotary_clk.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        let rotary_encoder = Rotary::new(rotary_dt, rotary_clk);

        let switch_pin = pins.gpio2.into_mode();
        if let Ok(switch_pin_low_on_init) = switch_pin.is_low() {
            if switch_pin_low_on_init {
                reset_to_bootsel();
            }
        }
        switch_pin.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);

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

        let (hid_producer, hid_consumer) = cx.local.hid_queue.split();
        usb_hid_task::spawn().ok();
        (
            Shared {
                usb_hid,
                usb_device,
                hid_consumer,
                hid_producer,
                switch_pin,
                rotary_encoder,
            },
            Local { led },
            init::Monotonics(Rp2040Monotonic::new(cx.device.TIMER)),
        )
    }

    fn reset_to_bootsel() -> ! {
        // For usb_boot to work, XOSC needs to be running
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

    #[task(binds = IO_IRQ_BANK0, priority = 2, shared = [rotary_encoder, switch_pin])]
    fn on_gpio(cx: on_gpio::Context) {
        // We skip checking the interrupt origin and simply clear all our GPIO interrupts,
        // and trigger a HID report whenever the pin state changes.
        // Checking the origin is more logically sound, but it does not save a considerable amount of
        // cycles while complicating the code.
        (cx.shared.switch_pin, cx.shared.rotary_encoder).lock(|switch_pin, rotary_encoder| {
            // Clear switch event and dispatch handler
            switch_pin.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
            process_switch_event::spawn_after(2.millis()).ok(); 

            // Clear rotary event and dispatch handler
            let pins = rotary_encoder.pins();
            pins.0.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            pins.0.clear_interrupt(hal::gpio::Interrupt::EdgeLow);

            pins.1.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            pins.1.clear_interrupt(hal::gpio::Interrupt::EdgeLow);

            process_rotation_event::spawn_after(1.micros()).ok(); 
        });
    }
    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_device, usb_hid])]
    fn on_usb(cx: on_usb::Context) {
        debug!("Entered USB IRQ");
        (cx.shared.usb_device, cx.shared.usb_hid).lock(|usb_device, usb_hid| {
            usb_device.poll(&mut [usb_hid]);
        });
    }

    #[task(shared = [switch_pin, hid_producer])]
    fn process_switch_event(cx: process_switch_event::Context) {
        (cx.shared.switch_pin, cx.shared.hid_producer).lock(
            |switch_pin, hid_producer| {
                // Assemble volume control report
                let switch_pin_state = switch_pin.is_low().unwrap_or(false);
                
                // Note - since we dont seem to catch the release event each time (even with triggers on both edges)
                // we do send a clear (all false) report after each event being sent.
                if switch_pin_state {
                    hid_producer.enqueue(VolumeControl {
                        mute: true,
                        volume_increment: false,
                        volume_decrement: false,
                    }).ok();

                    hid_producer.enqueue(VolumeControl {mute: false, volume_increment: false, volume_decrement: false }).ok();
                }
            }
        );
    }

    #[task(shared = [rotary_encoder, hid_producer])]
    fn process_rotation_event(cx: process_rotation_event::Context) {
        (cx.shared.rotary_encoder, cx.shared.hid_producer).lock(
            |rotary_encoder, hid_producer| {
                // Assemble volume control report
                let direction = rotary_encoder.update().unwrap_or(Direction::None);
                
                hid_producer.enqueue(VolumeControl {
                    mute: false,
                    volume_increment: direction == Direction::Clockwise,
                    volume_decrement: direction == Direction::CounterClockwise,
                }).ok();
                hid_producer.enqueue(VolumeControl {mute: false, volume_increment: false, volume_decrement: false }).ok();

            }
        );
    }

    #[task(shared = [usb_device, usb_hid, hid_consumer])]
    fn usb_hid_task(mut cx: usb_hid_task::Context) {
        // This task is responsible for emitting HID events in accordance
        // with the USB polling interval.
        // A limited backlog of events may be queued.
        let mut hid_consumer = cx.shared.hid_consumer;

        if let Some(report) = hid_consumer.lock(|h| h.dequeue()) {
            // Event was queued by encoder task, send to USB.
            debug!(
                "Dequeued event.",
            );
            if let Err(_err) = cx
                .shared
                .usb_hid
                .lock(|usb_hid| usb_hid.interface().write_report(&report))
            {
                error!("Error sending USB packet. {:?}", Debug2Format(&_err));
            } else {
                #[cfg(debug_assertions)]
                {
                    debug!("Sent USB packet succesfully.");
                    toggle_led::spawn(true).ok(); // Blink the led
                }
            }
        }
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
