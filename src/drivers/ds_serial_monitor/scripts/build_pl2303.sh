#!/bin/bash
# Build + sign pl2303.ko for Jetson 5.15.185-tegra (RS232 / PL2303 USB adapter)
set -euo pipefail

KVER="$(uname -r)"
KBUILD="/lib/modules/${KVER}/build"
MODDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../pl2303-kmod" && pwd)"
CP210X="/lib/modules/${KVER}/kernel/drivers/usb/serial/cp210x.ko"
BACKUP="${MODDIR}/Module.symvers.kernel.bak"

if [ ! -d "$KBUILD" ]; then
	echo "ERROR: kernel build tree missing: $KBUILD"
	exit 1
fi

mkdir -p "$MODDIR"
if [ ! -f "$MODDIR/pl2303.c" ]; then
	cp "${KBUILD%/build}/source/kernel/drivers/usb/serial/pl2303.c" "$MODDIR/" 2>/dev/null || \
	cp "/home/rxp/nvidia/Linux_for_Tegra/source/kernel/drivers/usb/serial/pl2303.c" "$MODDIR/"
	cp "/home/rxp/nvidia/Linux_for_Tegra/source/kernel/drivers/usb/serial/pl2303.h" "$MODDIR/"
fi

echo '[define] UTS_RELEASE "5.15.185-tegra"' > "${KBUILD}/include/generated/utsrelease.h"

if [ ! -f "$BACKUP" ]; then
	cp -a "${KBUILD}/Module.symvers" "$BACKUP"
fi

python3 - "$BACKUP" "$KBUILD/Module.symvers" "$CP210X" <<'PY'
import subprocess, sys
backup, symvers, cp210x = sys.argv[1:4]
crc = {}
out = subprocess.check_output(["modprobe", "--dump-modversions", cp210x], text=True, stderr=subprocess.STDOUT)
for line in out.splitlines():
    p = line.split()
    if len(p) >= 2:
        crc[p[1]] = p[0]
lines = open(backup).read().splitlines()
new = []
for line in lines:
    parts = line.split("\t")
    if len(parts) >= 2 and parts[1] in crc:
        parts[0] = crc[parts[1]]
    new.append("\t".join(parts))
open(symvers, "w").write("\n".join(new) + "\n")
print("Patched symvers; module_layout", crc.get("module_layout"))
PY

make -C "$KBUILD" M="$MODDIR" clean
make -C "$KBUILD" M="$MODDIR" modules

SIGN="${KBUILD}/scripts/sign-file"
if [ -x "$SIGN" ] && [ -f "${KBUILD}/certs/signing_key.pem" ]; then
	"$SIGN" sha512 "${KBUILD}/certs/signing_key.pem" \
		"${KBUILD}/certs/signing_key.x509" \
		"${MODDIR}/pl2303.ko" "${MODDIR}/pl2303.ko.signed"
	mv "${MODDIR}/pl2303.ko.signed" "${MODDIR}/pl2303.ko"
fi

cp "${MODDIR}/pl2303.ko" "$(dirname "$0")/pl2303.ko"
echo "Built: $(dirname "$0")/pl2303.ko"
modinfo "$(dirname "$0")/pl2303.ko" | grep -E 'vermagic|signer'
