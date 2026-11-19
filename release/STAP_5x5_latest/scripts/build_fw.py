# build_fw.py - PASS_MVDR_STAP (MOD13 firmware), STAP 5x5: one SD-card image
#
#   output/BOOT.bin   boots with 5 + 5 + 2 taps (per antenna + per antenna + image)
#
# Same bitstream / FSBL / firmware source as the other STAP package; the only
# difference is the build option GNSS_SMI_TAPS_BOOT (taps per antenna at boot,
# 0 = all 16). taps=1..16 on the console still changes it at run time.
#
# Run with the Vitis 2023.2 Python interpreter, from the package root:
#     vitis -s scripts/build_fw.py
#
#   1. fresh workspace build/ws
#   2. platform PASS_MVDR_STAP from hw/PASS_MVDR_STAP.xsa -> fsbl.elf
#   3. application STAP_5x5 (empty_application), sw/src imported, project
#      lscript.ld + UserConfig.cmake (-O0 -g3, libm) put back, GNSS_SMI_TAPS_BOOT
#   4. bootgen: fsbl + bitstream + app -> output/BOOT.bin

import glob
import os
import re
import shutil
import subprocess
import sys

import vitis

PLAT = "PASS_MVDR_STAP"
DOMAIN = "standalone_ps7_cortexa9_0"
CPU = "ps7_cortexa9_0"
#           app name      taps at boot (0 = all the bitstream has = 16)
VARIANTS = [("STAP_5x5", 5)]

ROOT = os.path.abspath(os.getcwd())
XSA = os.path.join(ROOT, "hw", PLAT + ".xsa")
SRC = os.path.join(ROOT, "sw", "src")
CFG = os.path.join(ROOT, "sw", "config")
WS = os.path.join(ROOT, "build", "ws")
OUT = os.path.join(ROOT, "output")


def die(msg):
    print("\n*** ERROR: " + msg)
    sys.exit(1)


def find_one(pattern, what):
    hits = sorted(glob.glob(pattern, recursive=True))
    if not hits:
        die("could not find " + what + " (" + pattern + ")")
    return hits[0]


def step(n, text):
    print("\n========== STEP %s: %s ==========" % (n, text), flush=True)


if not os.path.isfile(XSA):
    die("run this from the package root folder; missing " + XSA)
if not os.path.isfile(os.path.join(SRC, "main.c")):
    die("missing firmware sources in " + SRC)
if "GNSS_SMI_TAPS_BOOT" not in open(os.path.join(SRC, "gnss_smi.c")).read():
    die("sw/src/gnss_smi.c has no GNSS_SMI_TAPS_BOOT option - use this package's sw/src")

step(1, "fresh workspace " + WS)
if os.path.isdir(WS):
    shutil.rmtree(WS)
os.makedirs(WS)
os.makedirs(OUT, exist_ok=True)

client = vitis.create_client()
client.set_workspace(path=WS)

step(2, "platform " + PLAT + " from " + XSA)
platform = client.create_platform_component(name=PLAT, hw=XSA, os="standalone", cpu=CPU)
platform.build()
xpfm = find_one(os.path.join(WS, PLAT, "export", PLAT, PLAT + ".xpfm"), "platform .xpfm")
print("platform:", xpfm)

files = sorted(f for f in os.listdir(SRC) if os.path.isfile(os.path.join(SRC, f)))
elfs = {}
for app, taps in VARIANTS:
    step(3, "application %s (taps at boot: %s)" % (app, taps if taps else "16 = all"))
    comp = client.create_app_component(name=app, platform=xpfm, domain=DOMAIN)
    comp.import_files(from_loc=SRC, files=files, dest_dir_in_cmp="src")
    app_src = os.path.join(WS, app, "src")
    for f in ("lscript.ld", "UserConfig.cmake"):
        shutil.copyfile(os.path.join(CFG, f), os.path.join(app_src, f))
    if taps:
        ucfg = os.path.join(app_src, "UserConfig.cmake")
        txt = open(ucfg).read()
        txt, n = re.subn(r"set\(USER_COMPILE_DEFINITIONS\s*\n",
                         "set(USER_COMPILE_DEFINITIONS\nGNSS_SMI_TAPS_BOOT=%d\n" % taps, txt, count=1)
        if not n:
            die("USER_COMPILE_DEFINITIONS not found in UserConfig.cmake")
        open(ucfg, "w").write(txt)
        print("UserConfig.cmake: GNSS_SMI_TAPS_BOOT=%d" % taps)
    comp.build()
    elfs[app] = find_one(os.path.join(WS, app, "build", app + ".elf"), app + " .elf")
    print("application:", elfs[app])

vitis.dispose()

step(4, "bootgen -> output/BOOT.bin")
fsbl = find_one(os.path.join(WS, PLAT, "export", PLAT, "sw", "boot", "fsbl.elf"), "fsbl.elf")
bit = find_one(os.path.join(WS, PLAT, "**", "*.bit"), "bitstream")
bootgen = shutil.which("bootgen") or shutil.which("bootgen.bat") or shutil.which("bootgen.exe")
if not bootgen:
    die("bootgen not on PATH - run settings64 first")
shutil.copyfile(fsbl, os.path.join(OUT, "fsbl.elf"))
shutil.copyfile(bit, os.path.join(OUT, PLAT + ".bit"))
for app, taps in VARIANTS:
    d = OUT
    bif = os.path.join(d, "boot.bif")
    with open(bif, "w") as fh:
        fh.write("//arch = zynq; split = false; format = BIN\n")
        fh.write("the_ROM_image:\n{\n")
        fh.write("\t[bootloader]%s\n" % fsbl)
        fh.write("\t%s\n" % bit)
        fh.write("\t%s\n" % elfs[app])
        fh.write("}\n")
    shutil.copyfile(elfs[app], os.path.join(d, app + ".elf"))
    boot = os.path.join(d, "BOOT.bin")
    rc = subprocess.call([bootgen, "-arch", "zynq", "-image", bif, "-o", boot, "-w", "on"])
    if rc != 0 or not os.path.isfile(boot):
        die("bootgen failed for %s (exit %d)" % (app, rc))

print("\nDONE")
for app, taps in VARIANTS:
    print("  %-11s %s  (boots with %s taps per antenna + 2 image taps)"
          % (app, os.path.join(OUT, "BOOT.bin"), taps if taps else 16))
print("  bitstream  : " + os.path.join(OUT, PLAT + ".bit"))
