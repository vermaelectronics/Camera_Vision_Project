#!/usr/bin/env bash
# ============================================================================
#  bundle_full.sh  --  run ON YOUR PC: copy your ADI HDL tree into this package
#  (Vendor/adi_hdl) and zip the complete pipeline, so it builds on any PC that
#  has only Vivado / Vitis / Vitis HLS 2023.2 installed.
#
#    ./bundle_full.sh [ADI_HDL_DIR] [out.zip]
#      ADI_HDL_DIR  default: $ADI_HDL_DIR, else ~/antsdr/antsdr-fw-patch/plutosdr-fw/hdl
#      out.zip      default: next to the package folder, <package>_full.zip
#
#  Copied from the ADI tree: scripts/, projects/scripts/, projects/common/ (if
#  present), library/ (all IPs, including the packaged axi_vcxo_ctrl).
#  Left out: every other project, Vivado run/cache folders, logs and journals.
#  Build/ of this package is not zipped.
# ============================================================================
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/../.." && pwd)"
pkg="$(basename "$root")"
src="${1:-${ADI_HDL_DIR:-$HOME/antsdr/antsdr-fw-patch/plutosdr-fw/hdl}}"
out="${2:-$(dirname "$root")/${pkg}_full.zip}"

for f in scripts/adi_env.tcl projects/scripts/project-xilinx.mk library/axi_ad9361; do
  [[ -e "$src/$f" ]] || { echo "bundle_full: $src is not an ADI hdl tree ($f missing)"; exit 1; }
done
grep -rqs axi_vcxo_ctrl --include=component.xml "$src/library" || \
  { echo "bundle_full: no packaged axi_vcxo_ctrl IP in $src/library"; exit 1; }
[[ "$(cd "$src" && pwd)" == "$root/Vendor/adi_hdl" ]] && { echo "bundle_full: already bundled"; } || {
  dst="$root/Vendor/adi_hdl"
  rm -rf "$dst"; mkdir -p "$dst/projects"
  echo "bundle_full: copying $src -> Vendor/adi_hdl"
  ex=(--exclude='*.runs' --exclude='*.cache' --exclude='*.hw' --exclude='*.ip_user_files' --exclude='*.sim'
      --exclude='.Xil' --exclude='*.jou' --exclude='*.log' --exclude='*.str' --exclude='*.zip' --exclude='.git')
  for d in scripts library; do
    ( cd "$src" && tar "${ex[@]}" -cf - "$d" ) | ( cd "$dst" && tar -xf - )
  done
  for d in scripts common; do
    [[ -d "$src/projects/$d" ]] && ( cd "$src/projects" && tar "${ex[@]}" -cf - "$d" ) | ( cd "$dst/projects" && tar -xf - )
  done
  # top-level files the ADI makefiles include (Makefile, *.mk, quiet.mk ...)
  find "$src" -maxdepth 1 -type f \( -name '*.mk' -o -name 'Makefile' -o -name '*.tcl' -o -name 'LICENSE*' \) \
       -exec cp {} "$dst/" \;
  # the build checks for this project (it is only read, never changed)
  [[ -d "$src/projects/passthrough_lvds" ]] && ( cd "$src/projects" && tar "${ex[@]}" -cf - passthrough_lvds ) | ( cd "$dst/projects" && tar -xf - )
  echo "bundle_full: Vendor/adi_hdl = $(du -sh "$dst" | cut -f1)"
}
rm -f "$out"
( cd "$(dirname "$root")" && zip -qr -y "$out" "$pkg" -x "$pkg/Build/*" )
echo "bundle_full: $out ($(du -h "$out" | cut -f1))"
echo "bundle_full: on the other PC: unzip, source the three Xilinx settings64.sh, then"
echo "             cd $pkg/Automation/Linux && ./build_all.sh   (no ADI_HDL_DIR needed)"
