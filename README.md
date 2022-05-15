# RP2040 volume control - RTIC

This is a more intricate version of the [rp2040-volume-knob][https://github.com/0xa10/rp2040-volume-knob] using the RTIC framwork.
Everything is essentialy the same, except no more unsafe code and no more interrupt-free closures - the RTIC framework provides a dispatcher.

# Usage
## Deployment
The project is already configured with elf2uf2-rs as a runner, which means you can just plug in your Pico in update mode and run using 
```bash
cargo run
```

## Debugging
No UART output is available, the logs are printed out using defmt-rtt, which you can see using probe-run, but if you want to
debug too, you need to set up openocd, either on your own probe or on a Raspberry Pi (which is what I opted to do).
This allowed me to build, deploy and debug from my M1 Mac.

Follow section 5.1 in [Getting started with Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) to build openocd.

Hook up the SWD pins as following:
| Raspberry Pi host | Pico target |
|:-:|:-:|
| GND (pin 20) | SWD GND |
| GPIO24 (pin 18) | SWDIO |
| GPIO25 (pin 22) | SWDCLK |

Run openocd on the Raspberry Pi host, binding to an external interface for the remote debugging:
```bash 
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "bindto 0.0.0.0"
```

Now connect using gdb, setting the target binary and the remote (extended) debugging stub, setting RPI_IP to your Pi's address
```bash
gdb-multiarch -ex "target extended-remote $RPI_IP:3333" target/thumbv6m-none-eabi/debug/rp2040-volume-knob
```
Since I'm on an M1 mac with no gdb-multiarch, I use an Ubuntu VM with multipass, with my home folder mounted inside the VM.

You are now debugging the target, and can use monitor commands such as `monitor reset init` to restart the target, or `load` with our without a path to load a new .elf file.
To get defmt logs out - you need to extract the location of the RTT control block using the following command:
```bash
$ rust-nm -S target/thumbv6m-none-eabi/debug/rp2040-volume-knob | grep RTT # requires cargo-binutils
2003fae8 00000030 D _SEGGER_RTT
```
In the above case, the control block is as 0x2003fae8 and is 0x30 bytes in size.

Using this, we can ask (via gdb) the openocd monitor to set up a rtt source on a tcp address, using the following gdb commands:
```
monitor rtt server start 8765 0
monitor rtt setup 0x2003fae8 0x30 "SEGGER RTT"
monitor rtt start
```
At which points - the RTT server will be listening on the *RPi* (not localhost), and you can stream its output into
```bash
nc 192.168.1.148 8765 | defmt-print -e target/thumbv6m-none-eabi/debug/rp2040-volume-knob
```






