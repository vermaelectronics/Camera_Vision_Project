# build_fw.py - command-line build of the SFAP-PI firmware (app_sfap_lvds) and BOOT.bin
#
# Run with the Vitis 2023.2 Python interpreter (NOT normal python), from the
# package root (the folder that holds hw/, source/, scripts/):
#     vitis -s scripts/build_fw.py
#
#   1. fresh workspace build/ws
#   2. platform PASS_SFAP from hw/PASS_SFAP.xsa (standalone, ps7_cortexa9_0;
#      Vitis adds the zynq_fsbl boot domain), built -> fsbl.elf
#   3. application SFAP_LVDS_app (empty_application), sources from
#      source/Firmware/app_sfap_lvds; linker script: stack and heap 64 KiB
#      (the weight solver keeps 4 KiB arrays on the stack)
#   4. build -> SFAP_LVDS_app.elf
#   5. bootgen: fsbl.elf + bitstream + app -> output/BOOT.bin

import glob
import os
import re
import shutil
import subprocess
import sys

import vitis

PLAT = "PASS_SFAP"
APP = "SFAP_LVDS_app"
DOMAIN = "standalone_ps7_cortexa9_0"
CPU = "ps7_cortexa9_0"
STACK = "0x10000"
HEAP = "0x10000"

ROOT = os.path.abspath(os.getcwd())
XSA = os.path.join(ROOT, "hw", PLAT + ".xsa")
SRC = os.path.join(ROOT, "source", "Firmware", "app_sfap_lvds")
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
    die("missing " + XSA + " - run scripts/build_hw.sh first (or copy the XSA there)")
if not os.path.isfile(os.path.join(SRC, "main.c")):
    die("missing firmware sources in " + SRC)

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

step(3, "application " + APP)
comp = client.create_app_component(name=APP, platform=xpfm, domain=DOMAIN)
files = sorted(f for f in os.listdir(SRC) if os.path.isfile(os.path.join(SRC, f)))
comp.import_files(from_loc=SRC, files=files, dest_dir_in_cmp="src")
lscript = os.path.join(WS, APP, "src", "lscript.ld")
if os.path.isfile(lscript):
    txt = open(lscript).read()
    txt = re.sub(r"(_STACK_SIZE\s*=\s*DEFINED\(_STACK_SIZE\)\s*\?\s*_STACK_SIZE\s*:\s*)0x[0-9A-Fa-f]+", r"\g<1>" + STACK, txt)
    txt = re.sub(r"(_HEAP_SIZE\s*=\s*DEFINED\(_HEAP_SIZE\)\s*\?\s*_HEAP_SIZE\s*:\s*)0x[0-9A-Fa-f]+", r"\g<1>" + HEAP, txt)
    open(lscript, "w").write(txt)
    print("lscript.ld: stack", STACK, "heap", HEAP)
else:
    print("NOTE: no lscript.ld in the app src folder; stack/heap left at the template default")

step(4, "build " + APP)
comp.build()
elf = find_one(os.path.join(WS, APP, "build", APP + ".elf"), "application .elf")
print("application:", elf)

vitis.dispose()

step(5, "bootgen -> BOOT.bin")
fsbl = find_one(os.path.join(WS, PLAT, "export", PLAT, "sw", "boot", "fsbl.elf"), "fsbl.elf")
bit = find_one(os.path.join(WS, PLAT, "**", "*.bit"), "bitstream")
bif = os.path.join(OUT, "boot.bif")
with open(bif, "w") as fh:
    fh.write("//arch = zynq; split = false; format = BIN\n")
    fh.write("the_ROM_image:\n{\n")
    fh.write("\t[bootloader]%s\n" % fsbl)
    fh.write("\t%s\n" % bit)
    fh.write("\t%s\n" % elf)
    fh.write("}\n")
for src, name in ((fsbl, "fsbl.elf"), (bit, PLAT + ".bit"), (elf, APP + ".elf")):
    shutil.copyfile(src, os.path.join(OUT, name))

boot = os.path.join(OUT, "BOOT.bin")
bootgen = shutil.which("bootgen") or shutil.which("bootgen.bat") or shutil.which("bootgen.exe")
if not bootgen:
    die("bootgen not on PATH - run settings64 first, or run by hand:\n"
        "    bootgen -arch zynq -image output/boot.bif -o output/BOOT.bin -w on")
rc = subprocess.call([bootgen, "-arch", "zynq", "-image", bif, "-o", boot, "-w", "on"])
if rc != 0 or not os.path.isfile(boot):
    die("bootgen failed (exit %d)" % rc)

print("\nDONE")
print("  BOOT.bin : " + boot)
print("  app elf  : " + os.path.join(OUT, APP + ".elf"))
print("  fsbl     : " + os.path.join(OUT, "fsbl.elf"))
print("  bitstream: " + os.path.join(OUT, PLAT + ".bit"))
