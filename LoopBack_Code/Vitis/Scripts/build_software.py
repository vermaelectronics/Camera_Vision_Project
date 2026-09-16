#!/usr/bin/env python3
"""
build_software.py -- build the bare-metal application for the ANTSDR E310 V1.

WHY PYTHON AND NOT XSCT
    XSCT was REMOVED in Vitis 2026.1. Running it prints:
        [ERROR] ********** XSCT is disabled in Vitis 2026.1 release **********
    AMD's replacement is the Vitis Python API (`import vitis`), driven either
    interactively via `vitis -i` or as a script via `vitis -s <script.py>`.
    This script is the direct replacement for the old build_software.tcl.

WHAT IT DOES
    1. Creates a platform component from the XSA the FPGA build produced
       (requirement 15: hardware and software stay in step).
    2. Creates an empty standalone application on that platform.
    3. Imports the firmware from Source/Firmware (requirement 9: that stays the
       authoritative copy).
    4. Builds, and copies the ELF to Build/ELF.

    The workspace is regenerated every run, never patched, so it cannot drift
    from the XSA or from Source/ (requirements 11, 15).

INVOCATION
    vitis -s build_software.py -- <xsa> <workspace> <firmware_src> <elf_out>

SUCCESS MARKER
    Prints "SW_RESULT: PASS" only after an ELF actually exists on disk.
"""

import os
import shutil
import sys
import glob

PLATFORM_NAME = "e310_gnss_platform"
APP_NAME = "e310_gnss_app"
DOMAIN_NAME = "standalone_ps7_cortexa9_0"
CPU = "ps7_cortexa9_0"


def fail(msg, *evidence):
    print(f"SW_RESULT: FAIL - {msg}")
    for e in evidence:
        print(f"SW_EVIDENCE: {e}")
    sys.exit(2)


def parse_args():
    # `vitis -s script.py -- a b c` passes the tail through; be tolerant about
    # whether the separator survives.
    argv = sys.argv[1:]
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    if len(argv) < 4:
        fail("usage: build_software.py <xsa> <workspace> <firmware_src> "
             "<elf_out> [extra_defines]",
             f"got: {sys.argv}")
    paths = [os.path.abspath(a) for a in argv[:4]]
    # Optional 5th argument: comma-separated extra preprocessor defines WITHOUT
    # the -D, e.g. "GNSS_DEPLOY_AUTO_TX". Used by New-Deployment.ps1 to build
    # the unattended SD-card variant from the same sources, so the deployment
    # firmware is not a separate copy that can drift.
    extra = []
    if len(argv) >= 5 and argv[4].strip():
        extra = [d.strip() for d in argv[4].split(",") if d.strip()]
    return paths + [extra]


def main():
    xsa, workspace, fw_src, elf_out, extra_defines = parse_args()

    for path, what in ((xsa, "XSA"), (fw_src, "firmware source directory")):
        if not os.path.exists(path):
            fail(f"{what} not found", path)

    print(f"SW_STAGE: xsa       = {xsa}")
    print(f"SW_STAGE: workspace = {workspace}")
    print(f"SW_STAGE: firmware  = {fw_src}")

    if os.path.isdir(workspace):
        shutil.rmtree(workspace, ignore_errors=True)
    os.makedirs(workspace, exist_ok=True)

    import vitis  # provided by the Vitis Python environment

    client = vitis.create_client()
    client.set_workspace(workspace)

    # ---- platform ---------------------------------------------------------
    print("SW_STAGE: creating platform from the XSA")
    try:
        platform = client.create_platform_component(
            name=PLATFORM_NAME,
            hw_design=xsa,
            cpu=CPU,
            os="standalone",
            domain_name=DOMAIN_NAME,
            no_boot_bsp=True,
        )
    except Exception as exc:  # noqa: BLE001 - surface whatever the tool said
        fail("create_platform_component raised", repr(exc))

    print("SW_STAGE: building platform")
    try:
        platform.build()
    except Exception as exc:  # noqa: BLE001
        fail("platform build raised", repr(exc))

    xpfm = client.find_platform_in_repos(PLATFORM_NAME)
    if not xpfm:
        fail("platform was built but could not be found in the repositories",
             PLATFORM_NAME)
    print(f"SW_PLATFORM: {xpfm}")

    # ---- application ------------------------------------------------------
    print("SW_STAGE: creating application component")
    try:
        app = client.create_app_component(
            name=APP_NAME,
            platform=xpfm,
            domain=DOMAIN_NAME,
            template="empty_application",
        )
    except Exception as exc:  # noqa: BLE001
        fail("create_app_component raised", repr(exc))

    print(f"SW_STAGE: importing firmware from {fw_src}")
    try:
        app.import_files(from_loc=fw_src)
    except Exception as exc:  # noqa: BLE001
        fail("import_files raised", repr(exc))

    # The no-OS AD9361 driver needs these to compile for this board. They mirror
    # what app_config.h already defines; setting them explicitly means the build
    # does not depend on include order.
    # ---- build-variant defines -------------------------------------------
    #
    # These go into a GENERATED HEADER, not onto the compiler command line.
    #
    # set_app_config(key="USER_COMPILE_FLAGS", ...) is the obvious mechanism and
    # it DOES NOT WORK on Vitis 2026.1 -- it raises
    #     get_config_info: Unable to get the config information
    # This script used to print a note and carry on, which was harmless only
    # because the two flags involved (XILINX_PLATFORM, ANTSDR_E310) are also
    # defined in app_config.h. The first define that was NOT redundant
    # (GNSS_DEPLOY_AUTO_TX) silently failed to reach the compiler and produced a
    # "deployment" build that did not deploy anything. A generated header cannot
    # fail that way.
    #
    # The header is written into the WORKSPACE copy of the sources, so
    # Source/Firmware stays the authoritative, unmodified original.
    hdr_name = "gnss_build_defines.h"
    hdr_paths = glob.glob(os.path.join(workspace, "**", hdr_name), recursive=True)
    if not hdr_paths:
        fail(f"{hdr_name} not found in the imported sources",
             f"searched: {workspace}",
             f"it must exist in {fw_src} and be included by app_config.h")

    lines = [
        "/* GENERATED by build_software.py -- do not edit. */",
        "#ifndef GNSS_BUILD_DEFINES_H_",
        "#define GNSS_BUILD_DEFINES_H_",
        "",
    ]
    if extra_defines:
        for d in extra_defines:
            if "=" in d:
                name, _, value = d.partition("=")
                lines.append(f"#define {name.strip()} {value.strip()}")
            else:
                lines.append(f"#define {d} 1")
    else:
        lines.append("/* no build-variant defines */")
    lines += ["", "#endif /* GNSS_BUILD_DEFINES_H_ */", ""]

    for hp in hdr_paths:
        with open(hp, "w", encoding="ascii") as fh:
            fh.write("\n".join(lines))
        print(f"SW_DEFINES_HEADER: {hp}")

    if extra_defines:
        print(f"SW_EXTRA_DEFINES: {' '.join(extra_defines)}")
    else:
        print("SW_EXTRA_DEFINES: (none)")

    print("SW_STAGE: building application")
    try:
        app.build()
    except Exception as exc:  # noqa: BLE001
        fail("app build raised", repr(exc))

    # ---- collect the ELF --------------------------------------------------
    candidates = glob.glob(os.path.join(workspace, "**", "*.elf"), recursive=True)
    candidates = [c for c in candidates if APP_NAME in c] or candidates
    if not candidates:
        fail("build reported success but no ELF was produced",
             f"searched: {workspace}")

    elf = max(candidates, key=os.path.getmtime)
    os.makedirs(os.path.dirname(elf_out), exist_ok=True)
    shutil.copyfile(elf, elf_out)

    size = os.path.getsize(elf_out)
    print(f"SW_ELF_SOURCE: {elf}")
    print(f"SW_ELF: {elf_out} ({size} bytes)")
    print("SW_RESULT: PASS")

    try:
        vitis.dispose()
    except Exception:  # noqa: BLE001 - disposal failure must not fail the build
        pass


if __name__ == "__main__":
    main()
