#!/bin/bash
# Build pl2303 with symvers matched to LOADED kernel modules, install & load.
set -euo pipefail

KVER="$(uname -r)"
KBUILD="/lib/modules/${KVER}/build"
MODDIR="/home/rxp/catkin_ws/src/ds_serial_monitor/pl2303-kmod"
BACKUP="${MODDIR}/Module.symvers.kernel.bak"
SYMVERS="${KBUILD}/Module.symvers"
KO="${MODDIR}/pl2303.ko"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

log() { echo "[pl2303] $*"; }

[ -d "$KBUILD" ] || { log "ERROR: no kernel build at $KBUILD"; exit 1; }
[ -f "$MODDIR/pl2303.c" ] || {
	cp "$KBUILD/drivers/usb/serial/pl2303.c" "$KBUILD/drivers/usb/serial/pl2303.h" "$MODDIR/"
}

[ -f "$BACKUP" ] || cp -a "$SYMVERS" "$BACKUP"
echo '#define UTS_RELEASE "5.15.185-tegra"' > "${KBUILD}/include/generated/utsrelease.h"

cp -a "$BACKUP" "$SYMVERS"
make -C "$KBUILD" M="$MODDIR" clean >/dev/null 2>&1 || true
make -C "$KBUILD" M="$MODDIR" modules

log "Patching symvers for pl2303 imports ..."
python3 << PY
import subprocess, glob, os

kver = os.uname().release
backup = "${BACKUP}"
symvers = "${SYMVERS}"
ko = "${KO}"

needed = set()
for line in subprocess.check_output(["modprobe", "--dump-modversions", ko], text=True).splitlines():
    p = line.split()
    if len(p) >= 2 and p[0].startswith("0x"):
        needed.add(p[1])

loaded = set()
for line in open("/proc/modules"):
    loaded.add(line.split()[0].replace("-", "_"))

crc = {}
for path in glob.glob(f"/lib/modules/{kver}/**/*.ko", recursive=True):
    try:
        out = subprocess.check_output(
            ["modprobe", "--dump-modversions", path],
            text=True, stderr=subprocess.DEVNULL, timeout=2,
        )
    except Exception:
        continue
    modname = os.path.basename(path).replace(".ko", "").replace("-", "_")
    priority = 2 if modname in loaded else 1
    for line in out.splitlines():
        p = line.split()
        if len(p) >= 2 and p[0].startswith("0x") and p[1] in needed:
            if p[1] not in crc or priority >= crc[p[1]][0]:
                crc[p[1]] = (priority, p[0])

lines = open(backup).read().splitlines()
new_lines = []
for line in lines:
    parts = line.split("\t")
    if len(parts) >= 2 and parts[1] in crc:
        parts[0] = crc[parts[1]][1]
    new_lines.append("\t".join(parts))
open(symvers, "w").write("\n".join(new_lines) + "\n")
print(f"symbols needed: {len(needed)}, patched from loaded modules: {sum(1 for s in needed if s in crc)}")
for s in sorted(needed):
    print(f"  {s}: {crc[s][1] if s in crc else 'MISSING'}")
PY

make -C "$KBUILD" M="$MODDIR" clean >/dev/null 2>&1 || true
make -C "$KBUILD" M="$MODDIR" modules

SIGN="${KBUILD}/scripts/sign-file"
if [ -x "$SIGN" ] && [ -f "${KBUILD}/certs/signing_key.pem" ]; then
	"$SIGN" sha512 "${KBUILD}/certs/signing_key.pem" \
		"${KBUILD}/certs/signing_key.x509" "$KO" "${KO}.signed"
	mv "${KO}.signed" "$KO"
fi

cp "$KO" "${SCRIPT_DIR}/pl2303.ko"
mkdir -p "/lib/modules/${KVER}/extra"
cp "$KO" "/lib/modules/${KVER}/extra/pl2303.ko"
depmod -a

modprobe -r pl2303 2>/dev/null || true
if ! modprobe pl2303; then
	log "modprobe failed:"
	dmesg | tail -8
	exit 1
fi

sleep 2
cp -a "$BACKUP" "$SYMVERS"

log "SUCCESS - serial ports:"
ls -l /dev/ttyUSB* 2>/dev/null
for d in /dev/ttyUSB*; do
	[ -e "$d" ] || continue
	v=$(udevadm info -q property -n "$d" 2>/dev/null | grep '^ID_VENDOR=' | cut -d= -f2-)
	echo "  $d -> $v"
done
