# volume.control
<p align="center">
  <img align="center" width="600" alt="image" src="https://user-images.githubusercontent.com/12571311/176518111-1d384960-3244-4e97-bb6d-da6cd057f339.png">
</p>

A minimal implementation of a USB volume knob, utilizing a rotary encoder (volume increment, decrement) with a switch (mute).
This firmware targets the specially crafted _volume.control_ board, which is based on the [RP2040](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html) MCU.

The firmware itself is based solely on [embedded Rust](https://docs.rust-embedded.org/book), and makes use of the Real-Time Interrupt-Driven Concurrency, or [RTIC](https://rtic.rs) framework.

The host volume is controlled in the same way as it is controlled by keyboard media keys - by presenting the host with a USB-HID device, and dispatching Consumer Control HID reports indicating the Volume Increment `0xE9` , Volume Decrement `0xEA` and Mute `0xE2` Usage IDs.

USB-HID functionality is provided by @dlkj's [usbd-human-interface-device](https://github.com/dlkj/usbd-human-interface-device) library, and by implementing a minimal, single-byte report descriptor:

<p align="center">
  <img width="600" alt="image" src="https://user-images.githubusercontent.com/12571311/176538717-6e71ff1e-bda7-45d1-9e64-d1fc09fbefae.png">
</p>






## Flashing firmware (PICOBOOT)
The latest firmware release can be found in the [releases page](https://github.com/0xa10/volume.control/releases/latest).

To flash a new firmware to the _volume.control_ board - keep the mute button pressed while plugging the _volume.control_ into your host USB.
The device should boot into PICOBOOT mode, which allows for updates using [picotool](https://github.com/raspberrypi/picotool).

![flashing](https://user-images.githubusercontent.com/12571311/176535397-c8421733-3d41-4e91-8f2a-091a3b3415c3.gif)







## Building
Building 
```zsh
➜  ~ git clone https://github.com/0xa10/volume.control
➜  volume.control git:(main) ✗ cargo build --release                                        
    Finished release [optimized] target(s) in 0.03s
➜  volume.control git:(main) ✗ elf2uf2-rs target/thumbv6m-none-eabi/release/volume-control volume-control.uf2
```

## Recovery (BOOTSEL)
In case flashing fails catastrophically, or for some other reason the device does not boot into the application or PICOBOOT mode - it is possible to force it into BOOTSEL mode, by shorting the small pad on the top left corner of the board (in orange) to any ground pad (in black) with a paper clip or conductor of your choice.
<p align="center">
  <img width="700" alt="Screen Shot 2022-06-29 at 23 33 26" src="https://user-images.githubusercontent.com/12571311/176540134-5fb2eb1f-4bc4-4612-9694-0a9b7fd55e6c.png">  
</p>

> *pini//grigio*
>
> 2022
