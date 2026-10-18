#!/usr/bin/env bash
# 仅烧录 GD32 bootloader（OTA 复用 USART2 / PA2-PA3）。
# 使用方式：./flash_boot.sh
set -euo pipefail
BOOT="${1:-Bootloader/out/boot/Debug/Debug/boot.hex}"
if [ ! -f "$BOOT" ]; then echo "找不到 bootloader $BOOT，请先 ./build.sh" >&2; exit 1; fi
TMP="$(mktemp)"
cat > "$TMP" <<EOF
loadfile $(cd "$(dirname "$0")" && pwd)/$BOOT
r
g
exit
EOF
/usr/local/bin/JLinkExe -device STM32F407VG -if SWD -speed 1000 -autoconnect 1 -ExitOnError 1 -CommanderScript "$TMP"
rm -f "$TMP"
