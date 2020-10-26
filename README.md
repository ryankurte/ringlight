# Shitty RingLight

![Build](https://github.com/ryankurte/ringlight/workflows/Rust/badge.svg)

A low-cost and low-difficulty webcam ringlight based on a [3d-printed support](https://cad.onshape.com/documents/8d8e1210feb9aa656091cb68/w/f6f89561e16627434c69d84b/e/2f141cae78ba41f703d6d9aa), a seed studio [xiao-m0](https://wiki.seeedstudio.com/Seeeduino-XIAO), and a cheap [24-led WS2812 ring](https://www.aliexpress.com/item/33006920763.html?spm=a2g0s.9042311.0.0.535d4c4dpqgWA9).


## Getting Started

- [rlfw](rlfw/) contains the Xiao (SAMD21) ringlight firmware
- [rlclt](rlctl/) contains a ringlight control utility

The ringlight firmware runs on a seed xiao board, with WS2812 LEDs connected to pin 10. Firmware can be downloaded from the [releases](https://github.com/ryankurte/ringlight/releases/latest) page and installed via the uf2 bootloader. The ringlight control utility can be installed from source via `cargo install rlctl`, or downloaded from the [releases](https://github.com/ryankurte/ringlight/releases/latest) page.

Usage:

- `rlctl set-brightness BRIGHTNESS` to set brightness
- `rlctl set-rgb R G B` to set RGB colour output
- `rlctl help` to show help message / options

Note that values must be between `0-255` (or `0x00-0xff`).

### Linux / udev rules

On linux you will need a udev rule to provide your user with permission to access the device. If your account is a member of the `plugdev` group this may look like: `/etc/udev/40-ringlight.rules`:
```
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="fff1", MODE="0660", GROUP="plugdev"
```


## Development

### Dependencies

- `rust` with target `thumbv6m-none-eabi` (via `rustup target add thumbv6m-none-eabi`)
- `libusb` via `sudo apt install libusb-dev libusb-1.0`, `brew install libusb`, or `vcpkg install libsusb` (and maybe a `LIBUSB_DIR` setting on windows)


### Compiling / Flashing Firmware

Note that `memory.x` must be edited to remove the flash offset if your xiao-m0 does not currently have the [bootloader](https://github.com/Seeed-Studio/ArduinoCore-samd/blob/master/bootloaders/XIAOM0/bootloader-XIAO_m0-v3.7.0-33-g90ff611-dirty.bin) loaded.
In most cases you should be able to download the linked uf2 firmware above and copy this onto the disk presented by the xiao device. To enter bootloader mode you may need to short the GND/RESET pads on the back of the device twice in quick succession.

Other options:

- `cargo build --release` to build the binary
- `cargo flash --chip ATSAMD21G18A --speed 100 --release` to flash via a connected debugger
- `cargo hf2 --release --vid 0x2886 --pid 0x002f` to load via the bootloader (YMMV)


### Generating .uf2 compatible firmware files

`.uf2` files can be loaded onto the board by dragging and dropping into the correct file system.

Dependencies:

- `rustup component add llvm-tools-preview`
- `cargo install cargo-binutils u2fconv-rs` for creating `.uf2` bootloader files

Steps: 

- `cargo objcopy --bin rlfw --release -- -O binary rlfw.bin` to output a release binary file
- `uf2conv rlfw.bin --base 0x2000 --output rlfw.uf2 ` to generate a uf2 file


