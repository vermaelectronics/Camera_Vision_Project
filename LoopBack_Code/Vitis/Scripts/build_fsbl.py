#!/usr/bin/env python3
"""
build_fsbl.py -- build the Zynq-7000 First Stage Boot Loader for the E310 V1.

WHY THIS EXISTS
    Everything before 2026-09-15 loaded the bitstream and ELF over JTAG, which
    needs a PC, a cable and the Vitis tools. To run the loopback on an arbitrary
    bench the board has to boot itself, and on Zynq-7000 that means:

        BootROM -> FSBL -> (bitstream into the PL) -> application ELF

    all three packed into a single BOOT.BIN on a FAT32 SD card. The FSBL is the
    piece this project never needed before.

    The FSBL also replaces the ps7_init replay that jtag_program.py performs
    (ISSUE-0009): it brings up the PLLs, MIO, clocks and the DDR controller from
    the same ps7_init data, generated for THIS design, before loading anything.

HOW THE FSBL IS OBTAINED
    NOT by creating an application from the `zynq_fsbl` template on the
    standalone domain -- that fails, because the standalone BSP has no
    xilffs/xilrsa:
        "zynq_fsbl is not a valid template name for the given BSP.
         BSP is missing ['xilffs', 'xilrsa']."
    Instead, simply do NOT pass no_boot_bsp=True when creating the platform.
    Vitis then creates its own `zynq_fsbl` domain with the right BSP and builds
    fsbl.elf into the export tree as part of platform.build(). This script
    harvests that. (`zynqmp_fsbl` is for UltraScale+ and is NOT this part.)

INVOCATION
    vitis -s build_fsbl.py -- <xsa> <workspace> <elf_out>

SUCCESS MARKER
    Prints "FSBL_RESULT: PASS" only after an ELF actually exists on disk.
"""

import glob
import os
import shutil
import stat
import sys


def _force_remove(func, path, _exc):
    """rmtree error handler: clear the read-only bit and retry.

    The platform export tree contains READ-ONLY files (the generated SDT
    headers under export/.../hw/sdt/include). On Windows shutil.rmtree then
    fails with:
        [WinError 5] Access is denied: '...\\dt-bindings\\clock\\xlnx-versal-clk.h'
    which looks like a file lock but is not one. Clearing the attribute and
    retrying is the fix.
    """
    try:
        os.chmod(path, stat.S_IWRITE)
        func(path)
    except OSError:
        raise


def rmtree_force(path):
    """shutil.rmtree that copes with read-only files, on any Python 3.x.

    Python 3.12 renamed the rmtree error hook from `onerror` to `onexc` and
    deprecated the old name, so try the new one first.
    """
    try:
        shutil.rmtree(path, onexc=_force_remove)
    except TypeError:
        shutil.rmtree(path, onerror=_force_remove)

PLATFORM_NAME = "e310_fsbl_platform"
DOMAIN_NAME = "standalone_ps7_cortexa9_0"
CPU = "ps7_cortexa9_0"


def fail(msg, *evidence):
    print(f"FSBL_RESULT: FAIL - {msg}")
    for e in evidence:
        print(f"FSBL_EVIDENCE: {e}")
    sys.exit(2)


def parse_args():
    argv = sys.argv[1:]
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    if len(argv) < 3:
        fail("usage: build_fsbl.py <xsa> <workspace> <elf_out>",
             f"got: {sys.argv}")
    return [os.path.abspath(a) for a in argv[:3]]


def main():
    xsa, workspace, elf_out = parse_args()

    if not os.path.exists(xsa):
        fail("XSA not found", xsa)

    print(f"FSBL_STAGE: xsa       = {xsa}")
    print(f"FSBL_STAGE: workspace = {workspace}")

    # Remove the old workspace, and FAIL if it cannot be removed.
    #
    # ignore_errors=True here once left a PARTIALLY deleted tree behind, which
    # Vitis then refused with "Vitis IDE cannot recognize the workspace version"
    # -- an error that says nothing about the actual cause. A stale workspace is
    # usually a previous Vitis server still holding file locks.
    if os.path.isdir(workspace):
        try:
            rmtree_force(workspace)
        except OSError as exc:
            fail("could not remove the previous workspace",
                 f"{workspace}: {exc}",
                 "a previous Vitis server may still hold locks on it; "
                 "close other Vitis sessions and retry")
    if os.path.isdir(workspace):
        fail("workspace still exists after removal", workspace)
    os.makedirs(workspace, exist_ok=True)

    import vitis

    client = vitis.create_client()
    client.set_workspace(workspace)

    # The FSBL needs the BOOT BSP (xilffs for the SD filesystem, and the
    # ps7_init data), so unlike build_software.py this does NOT pass
    # no_boot_bsp=True.
    print("FSBL_STAGE: creating platform from the XSA")
    try:
        platform = client.create_platform_component(
            name=PLATFORM_NAME,
            hw_design=xsa,
            cpu=CPU,
            os="standalone",
            domain_name=DOMAIN_NAME,
        )
    except Exception as exc:  # noqa: BLE001
        fail("create_platform_component raised", repr(exc))

    print("FSBL_STAGE: building platform")
    try:
        platform.build()
    except Exception as exc:  # noqa: BLE001
        fail("platform build raised", repr(exc))

    xpfm = client.find_platform_in_repos(PLATFORM_NAME)
    if not xpfm:
        fail("platform was built but could not be found", PLATFORM_NAME)
    print(f"FSBL_PLATFORM: {xpfm}")

    # ---- harvest the FSBL the platform already built ----------------------
    #
    # Because no_boot_bsp is NOT set above, the platform build creates its own
    # `zynq_fsbl` domain (with the xilffs/xilrsa the FSBL needs) and builds
    # fsbl.elf into the export tree as part of platform.build().
    #
    # An earlier version of this script also called create_app_component(
    # template="zynq_fsbl") on the STANDALONE domain. That is redundant and it
    # fails, because the standalone domain's BSP has no xilffs/xilrsa:
    #     "zynq_fsbl is not a valid template name for the given BSP.
    #      BSP is missing ['xilffs', 'xilrsa']."
    # The FSBL lives in its own domain; do not try to build it in the
    # application's domain.
    print("FSBL_STAGE: locating the FSBL produced by the platform build")
    preferred = os.path.join(workspace, PLATFORM_NAME, "export",
                             PLATFORM_NAME, "sw", "boot", "fsbl.elf")
    if os.path.exists(preferred):
        elf = preferred
    else:
        candidates = glob.glob(os.path.join(workspace, "**", "fsbl.elf"),
                               recursive=True)
        if not candidates:
            fail("the platform built but no fsbl.elf was produced",
                 f"searched: {workspace}",
                 f"expected: {preferred}",
                 "check that no_boot_bsp is NOT set on the platform")
        elf = max(candidates, key=os.path.getmtime)
    os.makedirs(os.path.dirname(elf_out), exist_ok=True)
    shutil.copyfile(elf, elf_out)

    print(f"FSBL_ELF_SOURCE: {elf}")
    print(f"FSBL_ELF: {elf_out} ({os.path.getsize(elf_out)} bytes)")
    print("FSBL_RESULT: PASS")

    try:
        vitis.dispose()
    except Exception:  # noqa: BLE001
        pass


if __name__ == "__main__":
    main()
