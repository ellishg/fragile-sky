# fragile-sky
Display SF MUNI transit times on an epd2in13 e-ink display with an esp32c6 board

[![Rust](https://github.com/ellishg/fragile-sky/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/ellishg/fragile-sky/actions/workflows/build.yml)

https://docs.waveshare.com/ESP32-C6-ePaper-1.54

## Flashing
Connect to the board via the UART port.
```
cargo run
```

## Useful Links

* A flashing utility for esp32 boards
  * https://esp-rs.github.io/book/tooling/espflash.html
* Wifi
  * https://github.com/esp-rs/esp-wifi/blob/main/examples-esp32c6/examples/dhcp.rs
* esp32c6-hal docs
  * https://docs.rs/esp32c6-hal/latest/esp32c6_hal/index.html
* epd2in13 manual
  * https://www.waveshare.com/wiki/2.13inch_e-Paper_HAT_Manual
* epd-waveshare docs
  * https://docs.rs/epd-waveshare/latest/epd_waveshare/index.html
* Devkit
  * https://docs.espressif.com/projects/espressif-esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/user_guide.html
* Pins from a different board that may be useful
  * https://lastminuteengineers.com/esp32-pinout-reference/
