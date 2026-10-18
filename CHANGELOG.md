# Changelog

## [v1.0.2] - 2026-10-18

### Added

- Added a 32KB GD32F407 Bootloader at `0x08000000`; the App is linked at `0x08008000`.
- Added GD32 OTA triggered by Modbus holding register 49 (`0x4F54`), using USART2 PA2/PA3 at 115200 8N1.
- Added DMA+IDLE UART reception in the Bootloader, CRC16 validation, and a 64KB firmware staging limit.
- Added OTA failure reason codes in the internal flash status record.

### Changed

- Bootloader erases only the App flash sectors covered by the incoming image and checks erase/program results.
- `build.sh` builds Bootloader and App; `flash.sh` programs both, while `flash_boot.sh` updates only the Bootloader.
- ESP32 wiring for GD32 OTA is crossed: GPIO8 TX to PA3 RX, and PA2 TX to GPIO9 RX; share ground.

### Deployment

- First-time installation requires J-Link programming of both Bootloader and App with `./flash.sh`.
- GD32 OTA accepts `.bin` images up to 64KB; the image must be linked for `0x08008000`.
