#!/bin/bash
# Build (if needed) and install pl2303.ko for Jetson 5.15.185-tegra
set -e

KBUILD="${KBUILD:-/home/rxp/nvidia/Linux_for_Tegra/source/kernel}"
KVER="$(uname -r)"
KO_SRC="$KBUILD/drivers/usb/serial/pl2303.ko"
KO_DST="/lib/modules/$KVER/extra/pl2303.ko"
UTS="$KBUILD/include/generated/utsrelease.h"

if [ ! -d "$KBUILD" ]; then
	echo "ERROR: kernel source not found: $KBUILD"
	exit 1
fi

# Match running kernel vermagic (5.15.185-tegra)
if [ -f "$UTS" ]; then
	echo '#define UTS_RELEASE "5.15.185-tegra"' > "$UTS"
fi

if [ ! -f "$KO_SRC" ] || ! modinfo "$KO_SRC" 2>/dev/null | grep -q '5.15.185-tegra'; then
	echo "Building pl2303.ko ..."
	zcat /proc/config.gz > "$KBUILD/.config"
	"$KBUILD/scripts/config" --module CONFIG_USB_SERIAL_PL2303
	make -C "$KBUILD" ARCH=arm64 M=drivers/usb/serial CONFIG_USB_SERIAL_PL2303=m modules
fi

echo "Installing $KO_SRC -> $KO_DST"
mkdir -p "/lib/modules/$KVER/extra"
cp "$KO_SRC" "$KO_DST"
depmod -a

# Unbind/rebind USB if already plugged
modprobe -r pl2303 2>/dev/null || true
modprobe pl2303

echo ""
echo "Loaded pl2303. Serial devices:"
ls -l /dev/ttyUSB* 2>/dev/null || true
echo ""
udevadm info -q property -n /dev/ttyUSB5 2>/dev/null | grep -E 'ID_VENDOR|ID_MODEL|DEVNAME' || \
	udevadm info -q property -n /dev/ttyUSB0 2>/dev/null | grep -E 'ID_VENDOR|ID_MODEL' | head -4
echo ""
echo "STM32 RS232 port is usually the new Prolific ttyUSB (not Quectel 4G)."
echo "Run: ros2 run ds_serial_monitor find_serial_port"
