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

def xsa_for_sdt(src):
    """Vitis 2023.2 writes IP parameters from the XSA's .hwh straight into the
    system device tree (pl.dtsi). A binary literal such as the axi_tdd default
    polarity 0b010 (system_bd.tcl: TDD_DEFAULT_POL) becomes <0b010>, which dtc
    cannot parse ("pl.dtsi:139.30-31 syntax error"). Copy the XSA with every
    VALUE="0b..." in the .hwh written in decimal (same number). The bitstream
    and everything else in the XSA are copied unchanged."""
    import zipfile
    dst = os.path.join(ROOT, "build", PLAT + "_sdt.xsa")
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    pat = re.compile(r'(VALUE=")0[bB]([01]+)(")')
    changed = []
    with zipfile.ZipFile(src) as zin, zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED) as zout:
        for item in zin.infolist():
            data = zin.read(item.filename)
            if item.filename.lower().endswith(".hwh"):
                txt = data.decode("utf-8", errors="surrogateescape")
                def repl(m):
                    changed.append(item.filename + ": 0b" + m.group(2) + " -> " + str(int(m.group(2), 2)))
                    return m.group(1) + str(int(m.group(2), 2)) + m.group(3)
                txt = pat.sub(repl, txt)
                data = txt.encode("utf-8", errors="surrogateescape")
            zout.writestr(item, data)
    for c in changed:
        print("SDT fix:", c)
    if not changed:
        print("SDT fix: no binary literals in the .hwh (nothing changed)")
    return dst


def dtc_diagnose():
    """After a failed platform creation: show dtc's error and every device
    tree line whose <...> cells hold something dtc cannot read."""
    pp = os.path.join(WS, ".rigel_lopper", "system-top.dts.pp")
    dtc = shutil.which("dtc") or os.path.join(os.environ.get("XILINX_VITIS", ""), "bin", "dtc")
    if not os.path.isfile(pp):
        print("(no", pp, "to check)")
        return
    try:
        out = subprocess.run([dtc, "-I", "dts", "-O", "dtb", "-o", os.devnull, pp],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True).stdout
        print("dtc says:\n" + "\n".join(out.splitlines()[:5]))
    except Exception as e:
        print("could not run dtc:", e)
    tok_ok = re.compile(r'^(&[\w.-]+|0[xX][0-9a-fA-F]+|\d+|\(.*\))$')
    for n, line in enumerate(open(pp, errors="replace"), 1):
        m = re.search(r'=\s*<([^>]*)>', line)
        if m and any(not tok_ok.match(t) for t in m.group(1).split()):
            print("  bad cells, .pp line %d: %s" % (n, line.strip()))


step(2, "platform " + PLAT + " from " + XSA)
XSA_SDT = xsa_for_sdt(XSA)
try:
    platform = client.create_platform_component(name=PLAT, hw=XSA_SDT, os="standalone", cpu=CPU)
except Exception as e:
    print("\nPlatform creation failed:", e)
    dtc_diagnose()
    die("platform not created - send the lines above")
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

# Vitis 2023.2 SDT BSP: xparameters_ps.h defines XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ
# as XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ, which the generated xparameters.h does
# not define (it has XPAR_CPU_CORE_CLOCK_FREQ_HZ). xtime_l.h's COUNTS_PER_SECOND
# needs it, so gnss_aj.c and gnss_smi_agc.c (the two files that time with
# XTime) do not compile. Map it on the compiler line (same CPU clock value).
CPU_CLK_DEF = "XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ=XPAR_CPU_CORE_CLOCK_FREQ_HZ"
ucfg = os.path.join(WS, APP, "src", "UserConfig.cmake")
if os.path.isfile(ucfg):
    txt = open(ucfg).read()
    if CPU_CLK_DEF not in txt:
        txt, n = re.subn(r"set\(USER_COMPILE_DEFINITIONS\s*\n", "set(USER_COMPILE_DEFINITIONS\n" + CPU_CLK_DEF + "\n", txt, count=1)
        if n:
            open(ucfg, "w").write(txt)
            print("UserConfig.cmake: compile definition", CPU_CLK_DEF)
        else:
            print("WARNING: USER_COMPILE_DEFINITIONS not found in UserConfig.cmake; CPU clock define not added")
else:
    print("WARNING: no UserConfig.cmake in the app src folder; CPU clock define not added")

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
