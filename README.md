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
- **Memory**: FLASH @ 0x26000, RAM @ 0x20002000 (8KB offset for Embassy)
