# RAK4631 Rust USB CDC Example

Embassy async firmware for RAK4631 WisBlock (nRF52840) with USB CDC serial.

## Features

- LED blinks from boot (independent task)
- USB CDC serial with proper line endings
- DTR checking (only sends when terminal connected)
- **1200 baud touch reset to UF2 bootloader**

## Quick Start

```bash
cargo build --release
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/rak4631-hello rak4631-hello.bin
python3 uf2conv.py rak4631-hello.bin --base 0x26000 --family 0xADA52840 -o rak4631-hello.uf2
```

Flash: Double-tap reset, copy UF2 to `/Volumes/RAK4631`

Or auto-reset to bootloader: `stty -f /dev/cu.usbmodem123456781 1200`

## Usage

```bash
picocom -b 115200 /dev/cu.usbmodem123456781
```

Output:
```
Hello from nRF52840! Counter: 0
Hello from nRF52840! Counter: 1
...
```

## Hardware

- **Board**: RAK4631 WisBlock Core
- **LED**: P1.04 (blue, on base board)

## Memory Configuration

Custom `memory.x` required for RAK4631's Adafruit UF2 bootloader with SoftDevice S140 v6.1.1:

```
FLASH: 0x00026000 (bootloader + S140 v6.1.1 occupy first 152KB)
RAM:   0x20002000 (8KB offset required for Embassy framework)
```

Embassy's standard S140 v7.3.0 config uses 0x27000/0x20020000 with only 128KB RAM available. Our configuration provides 248KB RAM.
