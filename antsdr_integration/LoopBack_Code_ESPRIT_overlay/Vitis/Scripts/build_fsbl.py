#!/usr/bin/env python3
"""
build_fsbl.py -- build the Zynq first-stage boot loader (FSBL) for an XSA.

WHY
    build_software.py creates its platform with no_boot_bsp=True (the
    application needs no boot loader), so it never produces an FSBL. An SD-card
    BOOT.BIN needs one, and it must come from the SAME XSA as the bitstream:
    the FSBL carries that design's ps7_init (clocks, DDR, MIO).

INVOCATION
    vitis -s build_fsbl.py -- <xsa> <workspace> <fsbl_out.elf>

    <workspace> is wiped and regenerated; use a directory of its own (not the
    application workspace).

SUCCESS MARKER
    Prints "FSBL_RESULT: PASS" only after the ELF exists at <fsbl_out.elf>.
"""

import glob
import inspect
import os
import shutil
import sys

PLATFORM_NAME = "e310_fsbl_platform"
CPU = "ps7_cortexa9_0"


def fail(msg, *evidence):
    print(f"FSBL_RESULT: FAIL - {msg}")
    for e in evidence:
        print(f"FSBL_EVIDENCE: {e}")
    sys.exit(2)


# Same compatibility rule as build_software.py: keyword names differ between
# Vitis releases (hw= in 2023.2, hw_design= from 2024.1).
KW_ALIASES = {"hw_design": ["hw", "hw_design"], "hw": ["hw_design", "hw"]}


def call_compat(fn, **kwargs):
    try:
        params = inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return fn(**kwargs)
    if any(p.kind == inspect.Parameter.VAR_KEYWORD for p in params.values()):
        return fn(**kwargs)
    out = {}
    for key, val in kwargs.items():
        if key in params:
            out[key] = val
            continue
        alt = next((a for a in KW_ALIASES.get(key, []) if a in params), None)
        if alt:
            print(f"FSBL_COMPAT: {fn.__name__}: {key}= passed as {alt}=")
            out[alt] = val
        else:
            print(f"FSBL_COMPAT: {fn.__name__}: option {key}= not supported, skipped")
    return fn(**out)


def find_fsbl(workspace):
    found = [f for f in glob.glob(os.path.join(workspace, "**", "*.elf"), recursive=True)
             if "fsbl" in os.path.basename(f).lower() or "fsbl" in f.lower()]
    return max(found, key=os.path.getmtime) if found else None


def main():
    argv = sys.argv[1:]
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    if len(argv) < 3:
        fail("usage: build_fsbl.py <xsa> <workspace> <fsbl_out.elf>", f"got: {sys.argv}")
    xsa, workspace, fsbl_out = (os.path.abspath(a) for a in argv[:3])
    if not os.path.exists(xsa):
        fail("XSA not found", xsa)

    print(f"FSBL_STAGE: xsa       = {xsa}")
    print(f"FSBL_STAGE: workspace = {workspace}")
    if os.path.isdir(workspace):
        shutil.rmtree(workspace, ignore_errors=True)
    os.makedirs(workspace, exist_ok=True)

    import vitis  # provided by the Vitis Python environment

    client = vitis.create_client()
    client.set_workspace(workspace)

    # 1. a platform WITH its boot components: building it builds the FSBL
    print("FSBL_STAGE: creating platform with boot BSP (FSBL)")
    try:
        platform = call_compat(
            client.create_platform_component,
            name=PLATFORM_NAME,
            hw_design=xsa,
            cpu=CPU,
            os="standalone",
            domain_name="standalone_" + CPU,
            no_boot_bsp=False,
        )
    except Exception as exc:  # noqa: BLE001
        fail("create_platform_component raised", repr(exc))
    print("FSBL_STAGE: building platform")
    try:
        platform.build()
    except Exception as exc:  # noqa: BLE001
        fail("platform build raised", repr(exc))

    fsbl = find_fsbl(workspace)

    # 2. fallback: an explicit zynq_fsbl application on that platform
    if not fsbl:
        print("FSBL_STAGE: platform build gave no FSBL; trying a zynq_fsbl application")
        xpfm = glob.glob(os.path.join(workspace, "**", f"{PLATFORM_NAME}.xpfm"), recursive=True)
        if not xpfm:
            fail("platform .xpfm not found", workspace)
        last = None
        for domain in ("zynq_fsbl", "standalone_" + CPU):
            try:
                app = call_compat(client.create_app_component, name="fsbl_app",
                                  platform=xpfm[0], domain=domain, template="zynq_fsbl")
                app.build()
                break
            except Exception as exc:  # noqa: BLE001 - try the next domain
                last = exc
                print(f"FSBL_COMPAT: zynq_fsbl app on domain {domain} raised {exc!r}")
        fsbl = find_fsbl(workspace)
        if not fsbl:
            fail("no FSBL ELF produced", f"last error: {last!r}", f"searched: {workspace}")

    os.makedirs(os.path.dirname(fsbl_out), exist_ok=True)
    shutil.copyfile(fsbl, fsbl_out)
    print(f"FSBL_SOURCE: {fsbl}")
    print(f"FSBL_ELF: {fsbl_out} ({os.path.getsize(fsbl_out)} bytes)")
    print("FSBL_RESULT: PASS")
    try:
        vitis.dispose()
    except Exception:  # noqa: BLE001
        pass


if __name__ == "__main__":
    main()
