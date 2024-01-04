A Linux/Rust userspace driver for the Onsemi AX5043 transceiver.

Install Rust:
https://www.rust-lang.org/tools/install

Minimum Supported Rust Version (MSRV): 1.51? later? Check deps

The expected use case is to be run on an ARM based machine like a Raspberry Pi or Beaglebone so a Rust
crosscompiler setup will make development more convenient.

Install the cross toolchain: `rustup target add armv7-unknown-linux-gnueabihf`. !! Warning: The build libc must be <= target libc. Example error message:

``lib/arm-linux-gnueabihf/libc.so.6: version `GLIBC_2.33' not found``

On my setup my dev OS has a newer libc than Raspbian on the Pi and I'm unwilling to downgrade it.
Two options:
- Upgrade RPI to Bookworm (might be viable now but wasn't at the time)
- Install older version of LLVM crosscompiler package

I went with the latter - toolchain: https://github.com/abhiTronix/raspberry-pi-cross-compilers

Modify .cargo/config, runner.sh

Handy commands:
- `cargo run --example off --target armv7-unknown-linux-gnueabihf`
- `cargo r`
- `cargo b`
- `cargo c`
- Combine the above with `--examples` (plural) flag to apply the action to all the examples

Fresh Raspbian install
- https://www.raspberrypi.com/software/
- rpi-imager
  - Raspberry Pi OS (32-bit)
  - Internal SD Card Reader

On the SD card mounted on my laptop
- `touch <mountpoint>/boot/ssh` to enable ssh
- uncomment `dtparam=spi=on` in <mountpoint>/boot/config.txt to enable SPI0
- add `dtoverlay=spi1-1cs` to <moutpoint>/boot/config.txt to enable SPI1

See:https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#spi-overview

Laptop wired connection config: IPv4 -> Shared to other computers
Default IP on mine: 169.254.209.136
Default ssh login: pi@raspberrypi.local, pw:raspberry

Pinnout for Raspberry Pi 4B:
```
                     Pinnout:                       Colors:
                        J8                       VDD    -> Red
                       ----                      GND    -> Black
          TX VDD   1 - |oo| - 2                  IRQ    -> Gray
                   3 - |oo| - 4                  MOSI   -> Blue
                   5 - |oo| - 6                  MISO   -> Purple
                   7 - |oo| - 8                  SCLK   -> Green
          TX GND   9 - |oo| - 10                 SEI    -> Yellow
 (GPIO17) TX IRQ  11 - |oo| - 12 RX SEI (SPI1.0) SYSCLK -> Unused
                  13 - |oo| - 14
                  15 - |oo| - 16
          RX VDD  17 - |oo| - 18
 (SPI0.0) TX MISO 19 - |oo| - 20
 (SPI0.0) TX MOSI 21 - |oo| - 22
 (SPI0.0) TX SCLK 23 - |oo| - 24 TX SEI  (SPI0.0)
                  25 - |oo| - 26
                  27 - |oo| - 28
                  29 - |oo| - 30
                  31 - |oo| - 32
                  33 - |oo| - 34
 (SPI1.0) RX MISO 35 - |oo| - 36 RX IRQ  (GPIO16)
                  37 - |oo| - 38 RX MOSI (SPI1.0)
          RX GND  39 - |oo| - 40 RX SCLK (SPI1.0)
                       ----
                  GPIO EXPANSION
```
See https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-reduced-schematics.pdf

Libraries:
- https://www.kernel.org/doc/html/latest/spi/spidev.html
- https://www.kernel.org/doc/html/latest/driver-api/gpio/using-gpio.html

Other implemenations:
- https://notblackmagic.com/bitsnpieces/ax5043/
- https://gitlab.com/librespacefoundation/ax5043-driver
- https://github.com/oresat/oresat-firmware
- https://github.com/BrandenburgTech/DigitalTxRxRPi

List of docs (not committed for copywrite reasons):
- Datasheet
- Programming Manual
- ANs
- Demo board schematics

Examples
- `off`: Turns both radios off. Useful in case more complicated programs crash.
- `dump`: Resets radios and dumps all values. Useful for testing default values.
- `tx`: Simple transmit.
- `rx`: Simple receive.
- `ground`: Receives beacon packets on a socket and transmits them.
- `roundtrip`: Transmit from one radio and receive on the second.

Debian Packaging:
- Install [cargo-deb](https://github.com/kornelski/cargo-deb): `cargo install cargo-deb`
- Build the deb: `cargo deb --target armv7-unknown-linux-gnueabihf`
- The resulting package can be found in `target/armv7-unknown-linux-gnueabihf/debian/`


