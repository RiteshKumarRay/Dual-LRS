#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PIO_BIN="${PIO_BIN:-${HOME}/.platformio/penv/bin/pio}"

echo "=========================================================="
echo "Waiting for STM32 BlackPill in DFU mode (0483:df11)..."
echo "Instructions:"
echo "  1. Hold down the BOOT0 button on the BlackPill."
echo "  2. Press and release the NRST (Reset) button."
echo "  3. Release the BOOT0 button."
echo "=========================================================="

while true; do
    if lsusb | grep -qi "0483:df11"; then
        echo "Found STM32 DFU device!"
        break
    fi
    sleep 0.5
done

echo "Uploading dual_lrs_air firmware via DFU..."
"${PIO_BIN}" run -e dual_lrs_air -t upload -d "${REPO_ROOT}"

echo ""
echo "=========================================================="
echo "Firmware flashed successfully!"
echo "Now tap NRST (Reset) once to boot into Dual-LRS mode."
echo "Device will enumerate as /dev/ttyACM0."
echo "=========================================================="
