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

# gnss_passthrough register base. gnss_passthrough.h takes it from xparameters.h
# (XPAR_GNSS_PASSTHROUGH_0_BASEADDR / _BASEADDR / _S_AXI_BASEADDR) and otherwise
# falls back to 0x44A00000, which is NOT where the SFAP block design puts it
# (ad_cpu_interconnect 0x43C10000). Read the address from the XSA itself and,
# if the generated xparameters.h has none of those names, pass it to the compiler.
def pt_base_from_xsa(xsa):
    import zipfile
    with zipfile.ZipFile(xsa) as z:
        for n in z.namelist():
            if n.lower().endswith(".hwh"):
                t = z.read(n).decode("utf-8", errors="replace")
                for tag in re.findall(r"<MEMRANGE\b[^>]*>", t):
                    if 'INSTANCE="gnss_passthrough"' in tag:
                        m = re.search(r'BASEVALUE="(0[xX][0-9a-fA-F]+)"', tag)
                        if m:
                            return m.group(1)
    return None

PT_NAMES = ("XPAR_GNSS_PASSTHROUGH_0_BASEADDR", "XPAR_GNSS_PASSTHROUGH_BASEADDR",
            "XPAR_GNSS_PASSTHROUGH_S_AXI_BASEADDR")
xpar = os.path.join(WS, PLAT, "export", PLAT, "sw", DOMAIN, "include", "xparameters.h")
pt_xsa = pt_base_from_xsa(XSA)
pt_bsp = None
if os.path.isfile(xpar):
    xt = open(xpar).read()
    for nm in PT_NAMES:
        m = re.search(r"#define\s+" + nm + r"\s+(0[xX][0-9a-fA-F]+)", xt)
        if m:
            pt_bsp = (nm, m.group(1))
            break
print("gnss_passthrough base: XSA", pt_xsa, "| xparameters.h", pt_bsp if pt_bsp else "none")
PT_DEF = None
if pt_bsp:
    if pt_xsa and int(pt_bsp[1], 16) != int(pt_xsa, 16):
        die("xparameters.h says %s = %s but the XSA says %s" % (pt_bsp[0], pt_bsp[1], pt_xsa))
elif pt_xsa:
    PT_DEF = "XPAR_GNSS_PASSTHROUGH_BASEADDR=" + pt_xsa
else:
    die("gnss_passthrough base address not found in the XSA or in xparameters.h")

# Vitis 2023.2 SDT BSP: xparameters_ps.h defines XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ
# as XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ, which the generated xparameters.h does
# not define (it has XPAR_CPU_CORE_CLOCK_FREQ_HZ). xtime_l.h's COUNTS_PER_SECOND
# needs it, so gnss_aj.c and gnss_smi_agc.c (the two files that time with
# XTime) do not compile. Map it on the compiler line (same CPU clock value).
CPU_CLK_DEF = "XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ=XPAR_CPU_CORE_CLOCK_FREQ_HZ"
defs = [CPU_CLK_DEF] + ([PT_DEF] if PT_DEF else [])
ucfg = os.path.join(WS, APP, "src", "UserConfig.cmake")
if not os.path.isfile(ucfg):
    die("no UserConfig.cmake in " + os.path.dirname(ucfg) + " - cannot add the compile definitions " + ", ".join(defs))
txt = open(ucfg).read()
add = [d for d in defs if d not in txt]
if add:
    txt, n = re.subn(r"set\(USER_COMPILE_DEFINITIONS\s*\n", "set(USER_COMPILE_DEFINITIONS\n" + "\n".join(add) + "\n", txt, count=1)
    if not n:
        die("USER_COMPILE_DEFINITIONS not found in UserConfig.cmake - cannot add " + ", ".join(add))
    open(ucfg, "w").write(txt)
for d in defs:
    print("UserConfig.cmake: compile definition", d)

# Vitis 2023.2 SDT BSP has no xtime_l.h (gnss_aj.c and gnss_smi_agc.c include
# it), and its xiltimer XTime_GetTime() reads the SCU private timer: a 32-bit
# DOWN counter, while the firmware expects the classic xtime_l.h behaviour (the
# 64-bit global timer counting UP at CPU/2). Give the app its own xtime_l.h with
# exactly that behaviour -- the same global timer the classic header and the
# STAP firmware (gnss_time.h) read. No firmware source file is changed.
XTIME_SHIM = r'''/* xtime_l.h -- generated by scripts/build_fw.py for the Vitis 2023.2 SDT BSP.
 * Classic xtime_l.h behaviour: XTime = 64-bit Cortex-A9 global timer
 * (0xF8F00200), counting UP at CPU clock / 2. The SDT BSP's own XTime_GetTime()
 * (xiltimer) reads the 32-bit SCU private timer, which counts DOWN. */
#ifndef GNSS_XTIME_L_SHIM_H_
#define GNSS_XTIME_L_SHIM_H_
#include <stdint.h>
#include "xparameters.h"
#if defined(__has_include)
#  if __has_include("xiltimer.h")
#    include "xiltimer.h"          /* declares XTime / XTime_GetTime first */
#    define GNSS_XTIME_HAVE_TYPE 1
#  endif
#endif
#ifndef GNSS_XTIME_HAVE_TYPE
typedef uint64_t XTime;
#endif
#define GNSS_GTIMER_LO   (*(volatile uint32_t *)0xF8F00200U)
#define GNSS_GTIMER_HI   (*(volatile uint32_t *)0xF8F00204U)
#define GNSS_GTIMER_CTRL (*(volatile uint32_t *)0xF8F00208U)
static inline void gnss_xtime_get(XTime *t)
{
	uint32_t hi, lo;
	if (!(GNSS_GTIMER_CTRL & 1U))   /* not started by the BSP: start it */
		GNSS_GTIMER_CTRL |= 1U;
	do {                             /* consistent 64-bit read */
		hi = GNSS_GTIMER_HI;
		lo = GNSS_GTIMER_LO;
	} while (hi != GNSS_GTIMER_HI);
	*t = ((XTime)hi << 32) | lo;
}
#define XTime_GetTime(p) gnss_xtime_get(p)
#ifdef COUNTS_PER_SECOND
#undef COUNTS_PER_SECOND
#endif
#if defined(XPAR_CPU_CORE_CLOCK_FREQ_HZ)
#define COUNTS_PER_SECOND ((uint64_t)XPAR_CPU_CORE_CLOCK_FREQ_HZ / 2U)
#elif defined(XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ)
#define COUNTS_PER_SECOND ((uint64_t)XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2U)
#else
#define COUNTS_PER_SECOND (666666687ULL / 2U)
#endif
#endif
'''
shim = os.path.join(WS, APP, "src", "xtime_l.h")
if not os.path.isfile(os.path.join(SRC, "xtime_l.h")):
    with open(shim, "w") as fh:
        fh.write(XTIME_SHIM)
    print("xtime_l.h: written (64-bit global timer, COUNTS_PER_SECOND = CPU clock / 2)")

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
