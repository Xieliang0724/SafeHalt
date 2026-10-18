#!/usr/bin/env bash
# Flash bootloader + app to GD32F407VG via SEGGER J-Link.
set -euo pipefail
BOOT="${1:-Bootloader/out/boot/Debug/Debug/boot.hex}"
APP="${2:-out/SCH_NET-RELAY/Debug/Debug/SCH_NET-RELAY.hex}"
if [ ! -f "$BOOT" ]; then echo "找不到 bootloader $BOOT，请先 ./build.sh" >&2; exit 1; fi
if [ ! -f "$APP" ]; then echo "找不到 app $APP，请先 ./build.sh" >&2; exit 1; fi
printf 'device STM32F407VG\nsi SWD\nspeed 4000\nconnect\nloadfile "%s"\nloadfile "%s"\nr\ng\nexit\n' "$BOOT" "$APP" | /usr/local/bin/JLinkExe
