#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

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

    use usbd_hid::descriptor::generator_prelude::*;
    use usbd_hid::descriptor::MediaKeyboardReport;
    use usbd_hid::hid_class::HIDClass;

    use rotary_encoder_embedded::{Direction, RotaryEncoder};

	use heapless::spsc::{Consumer, Producer, Queue};

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Monotonic = Rp2040Monotonic;
    type LedPin = hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>;
    type SwitchPin = hal::gpio::Pin<hal::gpio::bank0::Gpio2, hal::gpio::PullUpInput>;
    type CLKPin = hal::gpio::Pin<hal::gpio::bank0::Gpio5, hal::gpio::PullUpInput>;
    type DTPin = hal::gpio::Pin<hal::gpio::bank0::Gpio6, hal::gpio::PullUpInput>;

	const HID_POLLING_INTERVAL_MS: u8 = 8;

    #[shared]
    struct Shared {
        usb_hid: HIDClass<'static, UsbBus>,
        usb_device: UsbDevice<'static, UsbBus>,
		hid_consumer: Consumer<'static, MediaKeyboardReport, 4>,
		hid_producer: Producer<'static, MediaKeyboardReport, 4>,
    }

    #[local]
    struct Local {
        led: LedPin,
        switch_pin: SwitchPin,
        rotary_encoder: RotaryEncoder<DTPin, CLKPin>,
    }

    #[init(local = [
			usb_bus: MaybeUninit<UsbBusAllocator<UsbBus>> = MaybeUninit::uninit(),
			hid_queue: Queue<MediaKeyboardReport, 4> = Queue::new(),
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
        let rotary_dt: DTPin = pins.gpio6.into_mode();
        rotary_dt.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);
        rotary_dt.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

        let rotary_clk: CLKPin = pins.gpio5.into_mode();
        rotary_clk.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);
        rotary_clk.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);
        let rotary_encoder = RotaryEncoder::new(rotary_dt, rotary_clk);
        let switch_pin = pins.gpio2.into_mode();
        switch_pin.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);
        let usb_bus: &'static _ = cx.local.usb_bus.write(UsbBusAllocator::new(UsbBus::new(
            cx.device.USBCTRL_REGS,
            cx.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));
        let usb_hid = HIDClass::new(usb_bus, MediaKeyboardReport::desc(), HID_POLLING_INTERVAL_MS);

        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x4853))
            .manufacturer("pini.grigio")
            .product("volume.control")
            .device_class(0x3)
            .max_packet_size_0(64)
            .max_power(500)
            .build();

		let (hid_producer, hid_consumer) = cx.local.hid_queue.split();
		usb_hid_feeder::spawn().ok();
        (
            Shared {
                usb_hid,
                usb_device,
				hid_consumer,
				hid_producer,
            },
            Local {
                led,
                switch_pin,
                rotary_encoder,
            },
            init::Monotonics(Rp2040Monotonic::new(cx.device.TIMER)),
        )
    }

    #[task(binds = IO_IRQ_BANK0, priority = 2, local = [rotary_encoder, switch_pin], shared = [hid_producer])]
    fn on_gpio(cx: on_gpio::Context) {
        let switch_pin = cx.local.switch_pin;
        let rotary_encoder = cx.local.rotary_encoder;
        let mut hid_producer = cx.shared.hid_producer;

        if switch_pin.interrupt_status(hal::gpio::Interrupt::EdgeLow) {
            if let Ok(switch_pin_state) = switch_pin.is_low() {
                if switch_pin_state {
                    hid_producer.lock(|p| p.enqueue(MediaKeyboardReport { usage_id: 0xE2 }).ok());
                }
            }
            switch_pin.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
        } else {
            rotary_encoder.update();
            let pins = rotary_encoder.borrow_pins();
            pins.0.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            pins.0.clear_interrupt(hal::gpio::Interrupt::EdgeLow);

            pins.1.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
            pins.1.clear_interrupt(hal::gpio::Interrupt::EdgeLow);

            match rotary_encoder.direction() {
                Direction::Clockwise => {
                    hid_producer.lock(|p| p.enqueue(MediaKeyboardReport { usage_id: 0xE9 }).ok());
                    hid_producer.lock(|p| p.enqueue(MediaKeyboardReport { usage_id: 0x0 }).ok());
                }
                Direction::Anticlockwise => {
                    hid_producer.lock(|p| p.enqueue(MediaKeyboardReport { usage_id: 0xEA }).ok());
                    hid_producer.lock(|p| p.enqueue(MediaKeyboardReport { usage_id: 0x0 }).ok());
                }
                Direction::None => {}
            }
        }
    }
    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_device, usb_hid])]
    fn on_usb(cx: on_usb::Context) {
        debug!("Entered USB IRQ");
        (cx.shared.usb_device, cx.shared.usb_hid).lock(|usb_device, usb_hid| {
            usb_device.poll(&mut [usb_hid]);
        });
    }

    #[task(shared = [usb_device, usb_hid, hid_consumer])]
    fn usb_hid_feeder(mut cx: usb_hid_feeder::Context) {
        let mut hid_consumer = cx.shared.hid_consumer;

		// Do a single event every poll interval.
		if let Some(report) = hid_consumer.lock(|h| h.dequeue()) {
			// Event was queued by encoder task, send to USB.
			debug!("Dequeued event, sending usage id: {:#02x}.", report.usage_id as u16);
			if let Err(_err) = cx
				.shared
				.usb_hid
				.lock(|usb_hid| usb_hid.push_input(&report))
			{
				error!("Error sending USB packet. {:?}", Debug2Format(&_err));
			} else {
				#[cfg(debug_assertions)] {
					debug!("Sent USB packet succesfully.");
					toggle_led::spawn(true).ok(); // Blink the led
				}
			}
		}

		usb_hid_feeder::spawn_after(8.millis()).ok(); // TODO - figure out millis type
    }

    #[task(priority = 1, local = [led])]
    fn toggle_led(cx: toggle_led::Context, blink: bool) {
        if blink {
            toggle_led::spawn_after(100.millis(), false).ok();
        }
        cx.local.led.toggle().ok();
    }
}
