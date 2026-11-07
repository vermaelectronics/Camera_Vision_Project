#!/usr/bin/env bash
# ============================================================================
#  BUILD_BOOT_BIN.sh  --  ONE CLICK: SFAP_snd2 sources -> Build/boot/BOOT.BIN
#                         (SFAP-PI v5.0, ANTSDR E310 V1, Vivado/Vitis 2023.2)
#
#  Does everything build_all.sh needs first, then runs it:
#    finds and sources Xilinx 2023.2 (Vivado, Vitis, Vitis HLS), checks the
#    version, checks host packages, sets SFAP_ROOT to this folder, gets the
#    ADI HDL library on the first run, then check -> hls -> ip -> hdl -> sw
#    -> boot.
#
#  USAGE (from anywhere; the package may be unzipped anywhere, no spaces)
#    ./BUILD_BOOT_BIN.sh                    full build (first run clones ADI hdl)
#    ./BUILD_BOOT_BIN.sh --adi <dir>        first run: copy this ADI_hdl_2023_r2
#                                           tree (e.g. the STAP project's) instead
#                                           of cloning it: faster
#    ./BUILD_BOOT_BIN.sh --xilinx <dir>     Xilinx install root (default: auto,
#                                           /tools/Xilinx, /opt/Xilinx, ~/Xilinx)
#    ./BUILD_BOOT_BIN.sh --from <stage>     resume at check|hls|ip|hdl|sw|boot
#    ./BUILD_BOOT_BIN.sh --skip-csim        skip the 9 HLS C-simulation scenarios
#    ./BUILD_BOOT_BIN.sh --clean            delete all build outputs first
#    ./BUILD_BOOT_BIN.sh --jtag             after the build, program + run over JTAG
#    ./BUILD_BOOT_BIN.sh --make-launcher [options]
#                                           put a double-click icon on the Desktop
#                                           (and in the app menu) that runs this
#                                           script with [options]
#
#  RESULT   Build/boot/BOOT.BIN   (log: Build/logs/one_click.log)
# ============================================================================
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi   # started with sh
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTO="$ROOT/Automation/Linux"
VER="2023.2"

adi_src=""; xil_root=""; clean=0; pause=0; make_launcher=0
pass=()            # options handed to build_all.sh
launcher_args=()   # options baked into the desktop launcher

need_arg() { [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || { echo "option $1 needs a value (see --help)"; exit 2; }; }
while [[ $# -gt 0 ]]; do
  case "$1" in
    --adi)           need_arg "$@"; adi_src="$2";  launcher_args+=("$1" "$2"); shift 2 ;;
    --xilinx)        need_arg "$@"; xil_root="$2"; launcher_args+=("$1" "$2"); shift 2 ;;
    --from|--only)   need_arg "$@"; pass+=("$1" "$2"); launcher_args+=("$1" "$2"); shift 2 ;;
    --skip-csim|--jtag) pass+=("$1"); launcher_args+=("$1"); shift ;;
    --clean)         clean=1; launcher_args+=("$1"); shift ;;
    --pause)         pause=1; shift ;;
    --make-launcher) make_launcher=1; shift ;;
    -h|--help)       sed -n '3,/^# =====/p' "$0" | sed '$d'; exit 0 ;;
    *)               echo "unknown option: $1 (see --help)"; exit 2 ;;
  esac
done

# Keep a launcher-opened terminal window open so the result can be read.
finish() {
  local rc=$1
  if [[ $pause -eq 1 ]]; then
    echo
    read -rp "Press Enter to close this window... " _ || true
  fi
  exit "$rc"
}

if [[ "$ROOT" =~ [[:space:]] ]]; then
  echo "ERROR: the package path contains a space: '$ROOT'"
  echo "       The ADI makefiles cannot handle that. Move the folder (e.g. to ~/SFAP/SFAP_snd2)."
  finish 2
fi

# ---------------------------------------------------------------------------
#  --make-launcher: write a double-clickable .desktop file and stop.
# ---------------------------------------------------------------------------
if [[ $make_launcher -eq 1 ]]; then
  for a in ${launcher_args[@]+"${launcher_args[@]}"}; do
    [[ "$a" =~ ^[A-Za-z0-9_./~+:=,-]+$ ]] || { echo "launcher: unsupported characters in '$a'"; exit 2; }
  done
  exec_line="$ROOT/BUILD_BOOT_BIN.sh --pause ${launcher_args[*]+${launcher_args[*]}}"
  desk="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
  [[ -n "$desk" && "$desk" != "$HOME" ]] || desk="$HOME/Desktop"
  apps="$HOME/.local/share/applications"
  mkdir -p "$desk" "$apps"
  for f in "$desk/SFAP_build_BOOT_BIN.desktop" "$apps/SFAP_build_BOOT_BIN.desktop"; do
    cat > "$f" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=SFAP: build BOOT.BIN
Comment=SFAP-PI E310 V1: sources to BOOT.BIN (Vivado/Vitis $VER)
Exec=${exec_line% }
Path=$ROOT
Terminal=true
Icon=utilities-terminal
Categories=Development;
EOF
    chmod +x "$f"
    gio set "$f" metadata::trusted true >/dev/null 2>&1 || true
    echo "launcher: $f"
  done
  echo "launcher: runs  ${exec_line% }"
  echo "launcher: on Ubuntu, if the icon shows 'untrusted', right-click it -> Allow Launching."
  exit 0
fi

# ---------------------------------------------------------------------------
#  --clean happens before logging starts (it deletes the log folder too).
# ---------------------------------------------------------------------------
export SFAP_ROOT="$ROOT"
user_jobs="${SFAP_JOBS:-}"     # env.sh sets a default; remember whether the user chose one
# shellcheck source=Automation/Linux/env.sh
source "$AUTO/env.sh"
if [[ $clean -eq 1 ]]; then
  echo "clean: removing $SFAP_BUILD, $SFAP_HDL_PRJ_DIR, $ROOT/Source/HLS/sfap_pi/build"
  rm -rf "$SFAP_BUILD" "$SFAP_HDL_PRJ_DIR" "$ROOT/Source/HLS/sfap_pi/build"
fi
mkdir -p "$SFAP_LOG_DIR"
LOG="$SFAP_LOG_DIR/one_click.log"
STAMP="$SFAP_BUILD/.one_click_start"

main() {
  local t0=$SECONDS
  say()  { printf '\n######## %s\n' "$*"; }
  fail() { printf '\n**** ONE-CLICK BUILD STOPPED: %s\n' "$*"; return 1; }

  say "SFAP_snd2 one-click build  ($(date '+%Y-%m-%d %H:%M:%S'))"
  echo "package : $ROOT"
  touch "$STAMP"

  # -- 1. package sanity --------------------------------------------------------
  [[ -f "$AUTO/build_all.sh" && -d "$ROOT/Source" ]] || { fail "this is not the SFAP_snd2 folder (Source/ or Automation/Linux/build_all.sh missing)"; return 1; }
  local f
  for f in "$AUTO"/*.sh "$AUTO"/*.tcl "$AUTO"/hdl_project/*; do
    if grep -q $'\r' "$f"; then sed -i 's/\r$//' "$f"; echo "fixed Windows line endings: ${f#$ROOT/}"; fi
  done
  chmod +x "$AUTO"/*.sh

  # -- 2. host packages ----------------------------------------------------------
  say "host checks"
  local missing=() t need=(make unzip flock sha256sum)
  # git only for the first-run clone of the ADI library
  if [[ ! -d "$ADI_HDL_DIR/library/axi_ad9361" && -z "$adi_src" ]]; then need+=(git); fi
  for t in "${need[@]}"; do
    command -v "$t" >/dev/null 2>&1 || missing+=("$t")
  done
  if [[ ${#missing[@]} -gt 0 ]]; then
    echo "missing: ${missing[*]}"
    echo "install : sudo apt install -y make git unzip util-linux coreutils"
    fail "host tools missing"; return 1
  fi
  echo "host tools: ok"
  if ! ldconfig -p 2>/dev/null | grep -q 'libtinfo\.so\.5'; then
    echo "WARNING: libtinfo.so.5 not found; xsct (sw stage) may fail on Ubuntu."
    echo "         fix: sudo apt install -y libtinfo5 libncurses5"
  fi
  local free_gb
  free_gb=$(df -Pk "$ROOT" | awk 'NR==2 {printf "%d", $4/1048576}')
  echo "free disk: ${free_gb} GB"
  [[ $free_gb -lt 10 ]] && echo "WARNING: less than 10 GB free; the ADI library + Vivado project need several GB."

  # -- 3. Xilinx 2023.2 ----------------------------------------------------------
  say "Xilinx $VER"
  local r p
  if [[ -z "$xil_root" ]]; then
    if command -v vivado >/dev/null 2>&1; then
      p="$(readlink -f "$(command -v vivado)")"            # <root>/Vivado/<ver>/bin/vivado
      r="$(dirname "$(dirname "$(dirname "$(dirname "$p")")")")"
      [[ -f "$r/Vivado/$VER/settings64.sh" ]] && xil_root="$r"
    fi
  fi
  if [[ -z "$xil_root" ]]; then
    for r in /tools/Xilinx /opt/Xilinx "$HOME/Xilinx" /Xilinx /opt/xilinx; do
      [[ -f "$r/Vivado/$VER/settings64.sh" ]] && { xil_root="$r"; break; }
    done
  fi
  if [[ -z "$xil_root" || ! -f "$xil_root/Vivado/$VER/settings64.sh" ]]; then
    fail "Vivado $VER not found (looked in /tools/Xilinx, /opt/Xilinx, ~/Xilinx). Re-run with --xilinx <install root>"; return 1
  fi
  echo "install root: $xil_root"
  for t in Vivado Vitis Vitis_HLS; do
    f="$xil_root/$t/$VER/settings64.sh"
    [[ -f "$f" ]] || { fail "$f not found: install Vitis $VER (it includes Vivado and Vitis HLS)"; return 1; }
    set +u
    # shellcheck disable=SC1090
    source "$f" >/dev/null 2>&1
    set -u
    echo "sourced: $f"
  done
  export XILINX_ROOT="$xil_root" XILINX_VER="$VER"
  for t in vivado vitis_hls xsct bootgen; do
    command -v "$t" >/dev/null 2>&1 || { fail "$t is not on PATH after sourcing the settings scripts"; return 1; }
  done
  local vv
  vv="$(vivado -version 2>/dev/null | head -1)"
  echo "vivado: $vv"
  if [[ "$vv" != *"v$VER"* ]]; then
    if [[ "${ADI_IGNORE_VERSION_CHECK:-}" == "1" ]]; then
      echo "WARNING: not Vivado $VER; continuing because ADI_IGNORE_VERSION_CHECK=1"
    else
      fail "Vivado $VER required (the ADI library hdl_2023_r2 is tied to it)"; return 1
    fi
  fi

  # -- 4. ADI HDL library ----------------------------------------------------------
  say "ADI HDL library"
  if [[ -d "$ADI_HDL_DIR/library/axi_ad9361" && -f "$ADI_HDL_DIR/scripts/adi_env.tcl" ]]; then
    echo "present: $ADI_HDL_DIR"
  elif [[ -n "$adi_src" ]]; then
    "$AUTO/setup_vendor.sh" "$adi_src" || { fail "could not copy the ADI tree from $adi_src"; return 1; }
  else
    echo "first run: cloning from GitHub (needs internet). To copy an existing tree instead: --adi <dir>"
    "$AUTO/setup_vendor.sh" || { fail "could not clone the ADI hdl library (network?)"; return 1; }
  fi

  # Parallel ADI library builds each start a Vivado: limit by RAM (~2 GB each).
  if [[ -z "$user_jobs" ]]; then
    local cpus mem_gb jobs
    cpus=$(nproc 2>/dev/null || echo 4)
    mem_gb=$(awk '/MemTotal/ {printf "%d", $2/1048576}' /proc/meminfo 2>/dev/null || echo 8)
    jobs=$(( mem_gb / 2 )); (( jobs > cpus )) && jobs=$cpus; (( jobs > 8 )) && jobs=8; (( jobs < 1 )) && jobs=1
    export SFAP_JOBS=$jobs
  fi
  echo "parallel library jobs: $SFAP_JOBS"

  # -- 5. the build ------------------------------------------------------------------
  say "build_all.sh ${pass[*]+${pass[*]}}  (check -> hls -> ip -> hdl -> sw -> boot; hdl takes 30-90 min)"
  local out="$SFAP_LOG_DIR/build_all.out"
  "$AUTO/build_all.sh" ${pass[@]+"${pass[@]}"} 2>&1 | tee "$out"
  local rc=${PIPESTATUS[0]}
  local mins=$(( (SECONDS - t0) / 60 ))
  if [[ $rc -ne 0 ]]; then
    local st
    st="$(grep -o 'BUILD FAILED in stage [a-z]*' "$out" | tail -1 | awk '{print $NF}')"
    if [[ -n "$st" ]]; then
      fail "stage '$st' failed after ${mins} min. Read $SFAP_LOG_DIR/$st.log, fix the cause, then resume with:
     ./BUILD_BOOT_BIN.sh --from $st"
    else
      fail "build_all.sh failed after ${mins} min (see $out)"
    fi
    return 1
  fi

  # -- 6. result ---------------------------------------------------------------------
  local bin="$SFAP_BOOT_DIR/BOOT.BIN"
  if [[ -f "$bin" && "$bin" -nt "$STAMP" ]]; then
    say "DONE in ${mins} min"
    ls -l "$bin"
    sha256sum "$bin"
    echo
    echo "BOOT.BIN : $bin"
    echo "SD card  : copy BOOT.BIN to a FAT32 SD card, boot jumpers on SD, console UART1 115200 8N1"
    echo "JTAG     : ./BUILD_BOOT_BIN.sh --only boot --jtag   (boot jumpers on JTAG)"
  else
    say "DONE in ${mins} min (the boot stage was not part of this run, so no new BOOT.BIN)"
  fi
  return 0
}

main 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
echo "full log: $LOG"
if [[ $rc -eq 0 && $pause -eq 1 && -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" && -f "$SFAP_BOOT_DIR/BOOT.BIN" ]]; then
  command -v xdg-open >/dev/null 2>&1 && xdg-open "$SFAP_BOOT_DIR" >/dev/null 2>&1 &
fi
finish "$rc"
