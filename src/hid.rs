use delegate::delegate;
use embedded_time::duration::Milliseconds;
use packed_struct::prelude::*;

use usb_device::bus::{InterfaceNumber, StringIndex, UsbBus};
use usb_device::class_prelude::DescriptorWriter;

use usbd_human_interface_device::hid_class::descriptor::HidProtocol;
use usbd_human_interface_device::hid_class::prelude::*;
use usbd_human_interface_device::interface::raw::{RawInterface, RawInterfaceConfig};
use usbd_human_interface_device::interface::{
    InterfaceClass, WrappedInterface, WrappedInterfaceConfig,
};
use usbd_human_interface_device::UsbHidError;

#[rustfmt::skip]
pub const VOLUME_CONTROL_DESCRIPTOR: &[u8] = &[
    0x05, 0x0C, //        Usage Page (Consumer Devices)  
    0x09, 0x01, //        Usage (Consumer Control)  
    0xA1, 0x01, //        Collection (Application)  
    0x05, 0x0C, //            Usage Page (Consumer Devices)  
    0x15, 0x00, //            Logical Minimum (0)  
    0x25, 0x01, //            Logical Maximum (1)  
    0x75, 0x01, //            Report Size (1)  
    0x95, 0x07, //            Report Count (3)  
    0x09, 0xE2, //            Usage (Mute)  
    0x09, 0xE9, //            Usage (Volume Increment)  
    0x09, 0xEA, //            Usage (Volume Decrement)  
    0x81, 0x02, //            Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)  
    0x95, 0x01, //            Report Count (5)  
    0x81, 0x01, //            Input (Const,Ary,Abs)  
    0xC0, //        End Collection
];

#[derive(Clone, Copy, Debug, PartialEq, PackedStruct)]
#[packed_struct(endian = "lsb", bit_numbering = "lsb0", size_bytes = "1")]
pub struct VolumeControlReport {
    #[packed_field(bits = "0")]
    pub mute: bool,
    #[packed_field(bits = "1")]
    pub volume_increment: bool,
    #[packed_field(bits = "2")]
    pub volume_decrement: bool,
}

pub struct VolumeControlInterface<'a, B: UsbBus> {
    inner: RawInterface<'a, B>,
}

impl<'a, B: UsbBus> VolumeControlInterface<'a, B> {
    pub fn write_report(&self, report: &VolumeControlReport) -> Result<(), UsbHidError> {
        // Explicitly map error to SerializationError
        let data = report.pack().map_err(|_| UsbHidError::SerializationError)?;

        self.inner
            .write_report(&data)
            .map_err(UsbHidError::from)
            .map(|_| ())
    }

    pub fn default_config() -> WrappedInterfaceConfig<Self, RawInterfaceConfig<'a>> {
        WrappedInterfaceConfig::new(
            RawInterfaceBuilder::new(VOLUME_CONTROL_DESCRIPTOR)
                .description("volume.control")
                .idle_default(Milliseconds(0))
                .unwrap()
                .in_endpoint(UsbPacketSize::Bytes8, Milliseconds(10))
                .unwrap()
                .build(),
            (),
        )
    }
}

impl<'a, B: UsbBus> InterfaceClass<'a> for VolumeControlInterface<'a, B> {
    delegate! {
        to self.inner{
           fn report_descriptor(&self) -> &'_ [u8];
           fn id(&self) -> InterfaceNumber;
           fn write_descriptors(&self, writer: &mut DescriptorWriter) -> usb_device::Result<()>;
           fn get_string(&self, index: StringIndex, _lang_id: u16) -> Option<&'_ str>;
           fn reset(&mut self);
           fn set_report(&mut self, data: &[u8]) -> usb_device::Result<()>;
           fn get_report(&mut self, data: &mut [u8]) -> usb_device::Result<usize>;
           fn get_report_ack(&mut self) -> usb_device::Result<()>;
           fn set_idle(&mut self, report_id: u8, value: u8);
           fn get_idle(&self, report_id: u8) -> u8;
           fn set_protocol(&mut self, protocol: HidProtocol);
           fn get_protocol(&self) -> HidProtocol;
        }
    }
}

impl<'a, B: UsbBus> WrappedInterface<'a, B, RawInterface<'a, B>> for VolumeControlInterface<'a, B> {
    fn new(interface: RawInterface<'a, B>, _: ()) -> Self {
        Self { inner: interface }
    }
}
