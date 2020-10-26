# Shitty RingLight

A low-cost and low-difficulty webcam ringlight based on a [3d-printed support](https://cad.onshape.com/documents/8d8e1210feb9aa656091cb68/w/f6f89561e16627434c69d84b/e/2f141cae78ba41f703d6d9aa), a seed studio [xiao-m0](https://wiki.seeedstudio.com/Seeeduino-XIAO), and a cheap [24-led WS2812 ring](https://www.aliexpress.com/item/33006920763.html?spm=a2g0s.9042311.0.0.535d4c4dpqgWA9).




## Status


- [rlfw](rlfw) contains the Xiao (SAMD21) ringlight firmware
- [rlclt](rlctl) contains a ringlight control utility


## Usage



## Development

### Dependencies

- `rust` with target `thumbv6m-none-eabi` (see `rustup target add`)


### Compiling / Flashing Firmware

Note that `memory.x` must be edited to remove the flash offset if your xiao-m0 does not currently have the [bootloader](https://github.com/Seeed-Studio/ArduinoCore-samd/blob/master/bootloaders/XIAOM0/bootloader-XIAO_m0-v3.7.0-33-g90ff611-dirty.bin) loaded.


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


