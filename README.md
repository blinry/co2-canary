# 🐤 CO2 Canary

An open-hardware, ultra-low power CO2 sensor.

## Features

- Battery life of at least 6 months
- Uses the [Senseair Sunlight CO2](https://senseair.com/product/sunlight-co2/)
    - Measurement range: 400-10000 ppm
    - Accuracy of +/- 50 ppm
    - Self-calibrating
- Shows a graph of the CO2 level over time
- Connectivity possible via Wifi, Bluetooth Low Energy, or Zigbee/Thread (not implemented yet)
- Connect a LiPo battery via the JST PH 2.0 connector, and charge via USB-C
    - Or connect your own power source to the *BAT* and *GND* pins

## Hardware

You can find the hardware design files in the [hardware](hardware) directory. The design was made using KiCad, and I've been using JLCPCB for manufacturing.

I'm hoping to write detailed instructions for how to order and assemble the device here soon. Until the following problems are fixed, I'd suggest you wait before you order one.

### Known problems for v0.1.0

- The battery level indicator chip is not connected correctly – it will not work as-is.
- The *reset* and *boot* buttons are not labelled on the board.

## Firmware

The firmware is located in the [firmware](firmware) directory. It's written in [no-std](https://docs.rust-embedded.org/book/intro/no-std.html) Rust, using the [esp-hal](https://github.com/esp-rs/esp-hal) crate.

## Acknowledgements

Many people helped me while working on this project! I'm deeply grateful for your support, this project would not have been possible without you.

## Licenses

The CO2 Canary is open hardware:

- The hardware design is licensed under the [CERN Open Hardware Licence v2](https://ohwr.org/cern_ohl_s_v2.txt) or any later version.
- The firmware is licensed under the [GNU General Public Licence v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html) or any later version.
