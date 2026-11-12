#!/usr/bin/env bash
#
# ANTSDR firmware build helper - Vivado/Vitis 2023.2, ADI plutosdr-fw v0.39
#
# Default target is the ANTSDR E310 (TARGET=ant). E200 / E310v2 work the same
# way with TARGET=e200 / TARGET=e310v2. For TARGET=ant the firmware also
# includes the GNSS anti-jam (RX1+RX2 -> FPGA -> TX1, AD9361 LVDS 2R2T): the
# SFAP-PI space-frequency filter (crpa6: 128 frequency bins, SFAP_BINS=64 for
# 64) driven by the 4.9.14 firmware as the gnss_crpa service with its "gnss"
# console (BUILD_E310.md sections 7 and 8).
#
#   ./build.sh deps       install Ubuntu build packages (uses sudo apt-get)
#   ./build.sh check      check host tools and Vivado/Vitis 2023.2
#   ./build.sh fetch      clone plutosdr-fw v0.39 + submodules at pinned commits
#   ./build.sh patch      apply the ANTSDR patches (safe to re-run)
#   ./build.sh hls        Vitis HLS: the sfap_pi anti-jam filter IP (ant only)
#   ./build.sh hdl        Vivado only: build/system_top.xsa (+ bitstream)
#   ./build.sh fsbl       Vitis/xsct: build/sdk/fsbl/Release/fsbl.elf
#   ./build.sh firmware   full firmware: boot.bin, ant.frm/.dfu, boot.frm/.dfu ...
#   ./build.sh sdimg      SD-card image: build_sdimg/BOOT.bin + uImage + ...
#   ./build.sh baremetal  design A WITHOUT Linux (ant only): FPGA + the SMI-PI
#                         firmware on bare metal -> build_baremetal/BOOT.bin
#   ./build.sh all        check + fetch + patch + hls + firmware + sdimg
#   ./build.sh all-baremetal  check + fetch + patch + hls + baremetal (no Linux)
#   ./build.sh outputs    list the generated files
#   ./build.sh reset      restore pristine v0.39 sources (drops patches and builds)
#   ./build.sh make ARGS  run plutosdr-fw's make with the right environment
#
# Environment overrides:
#   TARGET           ant (E310, default) | e200 | e310v2
#   XILINX_DIR       Xilinx install root (auto: /tools/Xilinx, /opt/Xilinx, ~/Xilinx)
#   VIVADO_SETTINGS  settings script sourced for Xilinx tools (default: ./xilinx_env.sh)
#   SKIP_LEGAL=1     skip buildroot legal-info (faster; no legal-info tarball)
#   SFAP_BINS=128|64 FFT size of the SFAP-PI filter (default 128; remembered in
#                    .sfap_bins once set). FPGA core, gnss_passthrough and the
#                    firmware are all switched together; changing it later
#                    rebuilds the filter IP and the FPGA.
#   SFAP_SKIP_CSIM=1 hls: skip the 9 C-simulation scenarios (faster)
#   FORCE_HLS=1      hls: rebuild the sfap_pi IP even if it is up to date
#   BM_DEFINES=...   baremetal: firmware build defines (default GNSS_LOG_SD;
#                    BM_DEFINES= (empty) builds without SD logging / saving)

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW_DIR="$ROOT/plutosdr-fw"
PATCH_DIR="$ROOT/patch"

# ADI's hdl scripts take their library folder from ADI_HDL_DIR / ADI_GHDL_DIR
# when these are set (e.g. left over from another ADI or SMI-PI project in
# ~/.bashrc). This build must use plutosdr-fw/hdl only.
for _v in ADI_HDL_DIR ADI_GHDL_DIR GNSS_CRPA_IP_REPO; do
	if [ -n "${!_v:-}" ]; then
		echo "build.sh: ignoring $_v=${!_v} (this build uses its own plutosdr-fw/hdl)" >&2
		unset "$_v"
	fi
done
unset _v

# SFAP-PI FFT size: the environment, else what an earlier run chose, else 128.
if [ -z "${SFAP_BINS:-}" ] && [ -f "$ROOT/.sfap_bins" ]; then
	SFAP_BINS="$(cat "$ROOT/.sfap_bins")"
fi
SFAP_BINS="${SFAP_BINS:-128}"
case "$SFAP_BINS" in
	128) SFAP_LOG2N=7 ;;
	64)  SFAP_LOG2N=6 ;;
	*) echo "build.sh: SFAP_BINS must be 128 or 64 (got '$SFAP_BINS')" >&2; exit 1 ;;
esac

# Everything below must reach make through the environment (see run_make).
export TARGET="${TARGET:-ant}"
export VIVADO_SETTINGS="${VIVADO_SETTINGS:-$ROOT/xilinx_env.sh}"
REQUIRED_XILINX_VERSION="2023.2"

PLUTOSDR_FW_URL="https://github.com/analogdevicesinc/plutosdr-fw.git"
PLUTOSDR_FW_REV="9e90bce43f849882cd43c20a08effca132790fb3"  # tag v0.39

# Submodule commits pinned by plutosdr-fw v0.39 (same as resetGit.sh).
SUBMODULES=(
	"hdl         065c8f186ef87ff049d279ed5859ee8d97d91808"
	"linux       f3da30df60047dc5a0b8fa8c640be774e0f784d9"
	"u-boot-xlnx 90401ce9ce029e5563f4dface63914d42badf5bc"
	"buildroot   e783aadccebad3413b3d60fcfe98a25eb395d839"
)

if [ -t 1 ]; then
	C_INFO=$'\e[1;34m' C_WARN=$'\e[1;33m' C_ERR=$'\e[1;31m' C_OK=$'\e[1;32m' C_END=$'\e[0m'
else
	C_INFO='' C_WARN='' C_ERR='' C_OK='' C_END=''
fi
info() { echo "${C_INFO}==>${C_END} $*"; }
ok()   { echo "${C_OK} ok${C_END} $*"; }
warn() { echo "${C_WARN}WARNING:${C_END} $*" >&2; }
die()  { echo "${C_ERR}ERROR:${C_END} $*" >&2; exit 1; }

usage() { awk 'NR > 2 && /^#/ { sub(/^# ?/, ""); print; next } NR > 2 { exit }' "${BASH_SOURCE[0]}"; }

check_target() {
	case "$TARGET" in
		ant|e200|e310v2) ;;
		*) die "TARGET='$TARGET' is not supported (use ant = E310, e200 or e310v2)" ;;
	esac
}

xilinx_run() {
	# Run a command with the Xilinx environment, like the Makefile does.
	bash -c "source '$VIVADO_SETTINGS' >/dev/null 2>&1 && $*"
}

# --------------------------------------------------------------------------
# deps
# --------------------------------------------------------------------------
cmd_deps() {
	command -v apt-get >/dev/null || die "apt-get not found: install the packages listed in BUILD_E310.md manually"
	local pkgs=(git build-essential fakeroot libncurses5-dev libssl-dev ccache
		dfu-util u-boot-tools device-tree-compiler mtools bc python3 cpio zip
		unzip rsync file wget bison flex libmpc-dev gawk patch perl locales)
	# Needed by Vivado/Vitis 2023.2 on Ubuntu 22.04; missing on newer releases.
	local optional=(libtinfo5 libncurses5 python-is-python3)
	local p
	for p in "${optional[@]}"; do
		if apt-cache show "$p" >/dev/null 2>&1; then
			pkgs+=("$p")
		else
			warn "package '$p' not available on this distribution (Vivado may need it, see BUILD_E310.md)"
		fi
	done
	info "sudo apt-get install -y ${pkgs[*]}"
	sudo apt-get update
	sudo apt-get install -y "${pkgs[@]}"
}

# --------------------------------------------------------------------------
# check
# --------------------------------------------------------------------------
cmd_check() {
	check_target
	local missing=0 t

	info "Host tools"
	for t in git make gcc g++ bison flex bc rsync cpio unzip zip wget file perl \
		python3 patch md5sum dtc mkimage; do
		if command -v "$t" >/dev/null 2>&1; then
			ok "$t"
		else
			echo "${C_ERR}missing${C_END} $t"
			missing=1
		fi
	done
	if [ ! -f /usr/include/openssl/ssl.h ]; then
		echo "${C_ERR}missing${C_END} OpenSSL headers (libssl-dev)"
		missing=1
	fi
	if command -v dfu-suffix >/dev/null 2>&1; then
		ok "dfu-suffix"
	else
		warn "dfu-suffix not found (apt install dfu-util): no .dfu files will be produced"
	fi
	if command -v dpkg >/dev/null 2>&1; then
		if dpkg -s gcc-arm-linux-gnueabihf >/dev/null 2>&1; then
			warn "gcc-arm-linux-gnueabihf is installed; the vendor recommends 'sudo apt-get purge gcc-arm-linux-gnueabihf'"
		fi
		if dpkg -s libfdt-dev >/dev/null 2>&1; then
			warn "libfdt-dev is installed; the vendor recommends 'sudo apt-get remove libfdt-dev'"
		fi
	fi
	[ "$missing" = 0 ] || die "install the missing host tools first (./build.sh deps)"

	info "Xilinx tools via VIVADO_SETTINGS=$VIVADO_SETTINGS"
	[ -f "$VIVADO_SETTINGS" ] || die "$VIVADO_SETTINGS does not exist"
	local vver
	vver="$(xilinx_run 'vivado -version 2>/dev/null | head -1' || true)"
	[ -n "$vver" ] || die "vivado not found after sourcing $VIVADO_SETTINGS (set XILINX_DIR, e.g. export XILINX_DIR=/tools/Xilinx)"
	case "$vver" in
		*"$REQUIRED_XILINX_VERSION"*) ok "vivado: $vver" ;;
		*) die "found '$vver' but Vivado $REQUIRED_XILINX_VERSION is required (ADI hdl refuses other versions)" ;;
	esac
	local tools=(xsct bootgen)
	[ "$TARGET" = ant ] && tools+=(vitis_hls)
	for t in "${tools[@]}"; do
		local p
		p="$(xilinx_run "command -v $t" || true)"
		if [ -n "$p" ]; then
			ok "$t: $p"
		else
			die "$t not found after sourcing $VIVADO_SETTINGS - install Vitis $REQUIRED_XILINX_VERSION (it provides xsct for the FSBL and vitis_hls for the anti-jam filter)"
		fi
	done

	info "Shell environment"
	if command -v vivado >/dev/null 2>&1 || command -v xsct >/dev/null 2>&1; then
		warn "Xilinx tools are already on PATH in this shell. Use a fresh terminal without"
		warn "sourcing settings64.sh: buildroot can break with Xilinx tools on PATH."
	else
		ok "Xilinx tools not on PATH (good: the Makefile sources them when needed)"
	fi
	if grep -qs "ADI_HDL_DIR\|ADI_GHDL_DIR" ~/.bashrc ~/.profile 2>/dev/null; then
		warn "$HOME/.bashrc or $HOME/.profile sets ADI_HDL_DIR/ADI_GHDL_DIR (another project); build.sh ignores it"
	fi
	[ -z "${LD_LIBRARY_PATH:-}" ] || warn "LD_LIBRARY_PATH is set; buildroot may refuse to build (unset LD_LIBRARY_PATH)"
	[ "$(id -u)" != 0 ] || warn "running as root: not needed, files will be owned by root"
	local free_gb
	free_gb=$(( $(df -Pk "$ROOT" | awk 'NR==2 {print $4}') / 1024 / 1024 ))
	if [ "$free_gb" -lt 40 ]; then
		warn "only ${free_gb} GB free in $ROOT; a full build needs about 40 GB"
	else
		ok "${free_gb} GB free disk space"
	fi
	ok "TARGET=$TARGET"
}

# --------------------------------------------------------------------------
# fetch
# --------------------------------------------------------------------------
cmd_fetch() {
	if [ ! -e "$FW_DIR/.git" ]; then
		info "Cloning plutosdr-fw into $FW_DIR"
		git clone "$PLUTOSDR_FW_URL" "$FW_DIR"
	fi
	if ! git -C "$FW_DIR" cat-file -e "${PLUTOSDR_FW_REV}^{commit}" 2>/dev/null; then
		git -C "$FW_DIR" fetch --tags origin
	fi
	if [ "$(git -C "$FW_DIR" rev-parse HEAD)" != "$PLUTOSDR_FW_REV" ]; then
		[ -z "$(git -C "$FW_DIR" status --porcelain --ignore-submodules=all)" ] ||
			die "plutosdr-fw is not at v0.39 and has local changes; run './build.sh reset' first"
		git -C "$FW_DIR" -c advice.detachedHead=false checkout -q "$PLUTOSDR_FW_REV"
	fi
	ok "plutosdr-fw at v0.39 ($PLUTOSDR_FW_REV)"

	local entry path sha url dir
	for entry in "${SUBMODULES[@]}"; do
		read -r path sha <<<"$entry"
		dir="$FW_DIR/$path"
		if [ -e "$dir/.git" ] && [ "$(git -C "$dir" rev-parse HEAD 2>/dev/null)" = "$sha" ]; then
			ok "$path already at $sha"
			continue
		fi
		if [ -e "$dir/.git" ] && [ -n "$(git -C "$dir" status --porcelain 2>/dev/null)" ]; then
			die "$path has local changes and is not at the pinned commit; run './build.sh reset' first"
		fi
		git -C "$FW_DIR" submodule init -- "$path" >/dev/null
		url="$(git -C "$FW_DIR" config --get "submodule.$path.url")"
		info "Fetching $path @ $sha from $url (shallow)"
		mkdir -p "$dir"
		if [ ! -e "$dir/.git" ]; then
			git -C "$dir" init -q
			git -C "$dir" remote add origin "$url"
		fi
		git -C "$dir" fetch --depth 1 origin "$sha" ||
			git -C "$dir" fetch origin "$sha" ||
			git -C "$dir" fetch origin ||
			die "could not fetch $path"
		git -C "$dir" -c advice.detachedHead=false checkout -q "$sha"
		[ "$(git -C "$dir" rev-parse HEAD)" = "$sha" ] || die "$path is not at $sha after checkout"
		ok "$path at $sha"
	done
}

# --------------------------------------------------------------------------
# patch
# --------------------------------------------------------------------------
apply_series() {
	# apply_series <dir> <patch>...  Apply patches in order, skipping those
	# already applied. Later patches may edit files an earlier one created,
	# so "already applied" is decided from the newest patch backwards: once
	# patch N reverses cleanly, patches 1..N are in place.
	local dir="$1"; shift
	local patches=("$@") n=$# start=0 i name p
	for p in "${patches[@]}"; do [ -f "$p" ] || die "missing patch file $p"; done
	for ((i = n - 1; i >= 0; i--)); do
		if git -C "$dir" apply --reverse --check "${patches[i]}" 2>/dev/null; then
			start=$((i + 1))
			break
		fi
	done
	for ((i = 0; i < n; i++)); do
		p="${patches[i]}"
		name="${p#"$PATCH_DIR"/}"
		if [ "$i" -lt "$start" ]; then
			ok "already applied $name"
		elif git -C "$dir" apply --check "$p" 2>/dev/null; then
			git -C "$dir" apply --whitespace=nowarn "$p"
			ok "applied         $name"
		else
			git -C "$dir" apply --check "$p" || true
			die "$name does not apply to ${dir#"$ROOT"/} (modified sources or another TARGET was patched); run './build.sh reset'"
		fi
	done
}

cmd_patch() {
	check_target
	[ -e "$FW_DIR/.git" ] || die "sources not fetched yet; run './build.sh fetch'"
	local entry path sha
	for entry in "${SUBMODULES[@]}"; do
		read -r path sha <<<"$entry"
		[ -e "$FW_DIR/$path/.git" ] || die "$path missing; run './build.sh fetch'"
	done
	info "Applying ANTSDR patches for TARGET=$TARGET"
	# 0002-* (ant only): GPS L1 RX1 -> FPGA -> TX1 loopback (gnss_passthrough
	# in the FPGA, /usr/sbin/gnss_loopback in Linux). See BUILD_E310.md.
	# 0003-* (ant only): anti-jam. hdl: AD9361 LVDS 2R2T, gnss_passthrough
	# v4.9.14 + the sfap_pi HLS core (0004: 64 bins instead of 128); linux: LVDS 2R2T device
	# tree; u-boot: mode=2r2t; buildroot: gnss-crpa package (gnss_crpa
	# service + "gnss" console, gnss_loopback forwards to it).
	local hdl_series=("$PATCH_DIR/0001-add-three-ant-hdl-support-bump-to-v0.39-v1.patch")
	local br_series=("$PATCH_DIR/$TARGET/0001-add-support-buildroot.patch")
	local ub_series=("$PATCH_DIR/$TARGET/0001-add-support-uboot.patch")
	local lx_series=("$PATCH_DIR/$TARGET/0001-add-support-linux.patch")
	local f
	for f in "$PATCH_DIR/$TARGET"/000[23]-hdl-*.patch; do [ -f "$f" ] && hdl_series+=("$f"); done
	for f in "$PATCH_DIR/$TARGET"/000[23]-buildroot-*.patch; do [ -f "$f" ] && br_series+=("$f"); done
	for f in "$PATCH_DIR/$TARGET"/0003-uboot-*.patch; do [ -f "$f" ] && ub_series+=("$f"); done
	for f in "$PATCH_DIR/$TARGET"/0003-linux-*.patch; do [ -f "$f" ] && lx_series+=("$f"); done
	if [ -f "$PATCH_DIR/$TARGET/0002-buildroot-add-gnss-loopback.patch" ]; then
		# An earlier version of the 0002 patch installed an older
		# gnss_loopback. Remove only what it added (the script and its
		# install line), so the current series applies without a reset.
		local br="$FW_DIR/buildroot" p applied=0
		for p in "${br_series[@]:1}"; do
			git -C "$br" apply --reverse --check "$p" 2>/dev/null && applied=1
		done
		if [ -f "$br/board/ant/gnss_loopback" ] && [ "$applied" = 0 ]; then
			info "Replacing an earlier gnss_loopback with the current version"
			rm -f "$br/board/ant/gnss_loopback"
			sed -i '/BOARD_DIR}\/gnss_loopback /d' "$br/board/ant/post-build.sh"
		fi
	fi
	# The 64-bin patch sits on top of 0003: take it off while the series is
	# checked (0003 would not look "already applied" under it); sfap_bins_apply
	# puts the wanted size back.
	sfap_bins_lift
	apply_series "$FW_DIR/hdl"         "${hdl_series[@]}"
	apply_series "$FW_DIR/u-boot-xlnx" "${ub_series[@]}"
	apply_series "$FW_DIR/linux"       "${lx_series[@]}"
	apply_series "$FW_DIR/buildroot"   "${br_series[@]}"
	apply_series "$FW_DIR"             "$PATCH_DIR/$TARGET/0001-add-support-makefile-scripts.patch"
	sfap_bins_apply
	sync_crpa
}

# --------------------------------------------------------------------------
# SFAP-PI FFT size in the hdl tree (ant only): 0004-hdl-sfap-64bins.patch on
# top of 0003 means 64 bins, without it 128.
# --------------------------------------------------------------------------
SFAP_PREV=""
sfap_bins_lift() {
	local p="$PATCH_DIR/$TARGET/0004-hdl-sfap-64bins.patch" hdl="$FW_DIR/hdl"
	[ -f "$p" ] || return 0
	if git -C "$hdl" apply --reverse --check "$p" 2>/dev/null; then
		git -C "$hdl" apply --reverse --whitespace=nowarn "$p"
		SFAP_PREV=64
	fi
}

sfap_bins_apply() {
	local p="$PATCH_DIR/$TARGET/0004-hdl-sfap-64bins.patch" hdl="$FW_DIR/hdl" have=128 prev
	[ -f "$p" ] || return 0
	git -C "$hdl" apply --reverse --check "$p" 2>/dev/null && have=64
	prev="${SFAP_PREV:-$have}"
	if [ "$have" != "$SFAP_BINS" ]; then
		if [ "$SFAP_BINS" = 64 ]; then
			git -C "$hdl" apply --whitespace=nowarn "$p" || die "cannot apply ${p#"$PATCH_DIR"/}; run './build.sh reset'"
		else
			git -C "$hdl" apply --reverse --whitespace=nowarn "$p" || die "cannot remove ${p#"$PATCH_DIR"/}; run './build.sh reset'"
		fi
	fi
	if [ "$prev" != "$SFAP_BINS" ]; then
		# the FPGA project does not notice a changed filter IP by itself
		if [ -f "$FW_DIR/build/system_top.xsa" ] || [ -d "$hdl/projects/ant/ant.runs" ]; then
			warn "SFAP-PI changed from $prev to $SFAP_BINS bins: the FPGA is built again"
			rm -f "$FW_DIR/build/system_top.xsa" "$FW_DIR/build/system_top.bit"
			make -C "$hdl/projects/ant" clean >/dev/null 2>&1 || true
		fi
	fi
	echo "$SFAP_BINS" > "$ROOT/.sfap_bins"
	ok "SFAP-PI: $SFAP_BINS frequency bins (gnss_passthrough, filter core and firmware)"
}

# Firmware sources with the FFT size of this build: <dir>/gnss_sfap_cfg.h
sfap_cfg_header() {
	sed -i "s/^#define AJ_LOG2N [0-9]*\$/#define AJ_LOG2N $SFAP_LOG2N/" "$1/gnss_sfap_cfg.h"
	grep -q "^#define AJ_LOG2N $SFAP_LOG2N\$" "$1/gnss_sfap_cfg.h" || die "cannot set AJ_LOG2N in $1/gnss_sfap_cfg.h"
}

# --------------------------------------------------------------------------
# gnss_crpa sources -> buildroot package (ant only)
# --------------------------------------------------------------------------
sync_crpa() {
	local dst="$FW_DIR/buildroot/package/gnss-crpa"
	[ -d "$dst" ] || return 0
	if [ -d "$dst/src" ] && diff -q "$ROOT/gnss_crpa/Makefile" "$dst/src/Makefile" >/dev/null 2>&1 &&
	   diff -rq -x gnss_sfap_cfg.h "$ROOT/gnss_crpa/src" "$dst/src/src" >/dev/null 2>&1 &&
	   diff -rq "$ROOT/gnss_crpa/linux" "$dst/src/linux" >/dev/null 2>&1 &&
	   grep -q "^#define AJ_LOG2N $SFAP_LOG2N\$" "$dst/src/src/gnss_sfap_cfg.h" 2>/dev/null; then
		ok "gnss_crpa sources up to date in buildroot/package/gnss-crpa/src ($SFAP_BINS bins)"
		return 0
	fi
	rm -rf "$dst/src"
	mkdir -p "$dst/src"
	cp -a "$ROOT/gnss_crpa/Makefile" "$ROOT/gnss_crpa/src" "$ROOT/gnss_crpa/linux" "$dst/src/"
	sfap_cfg_header "$dst/src/src"
	# buildroot copies a local package only once: make it build again
	rm -rf "$FW_DIR"/buildroot/output/build/gnss-crpa-*
	ok "gnss_crpa sources copied to buildroot/package/gnss-crpa/src"
}

# --------------------------------------------------------------------------
# hls: the sfap_pi space-frequency anti-jam filter (Vitis HLS), ant only
# --------------------------------------------------------------------------
cmd_hls() {
	check_target
	[ "$TARGET" = ant ] || { ok "hls: nothing to do for TARGET=$TARGET"; return 0; }
	[ -d "$FW_DIR/hdl/library" ] || die "sources not fetched yet; run './build.sh fetch' and './build.sh patch'"
	local src="$ROOT/hls/sfap_pi" out="$FW_DIR/hdl/library/antsdr_ip/sfap_pi"
	if [ -z "${FORCE_HLS:-}" ] && [ -f "$out/component.xml" ] &&
	   [ "$(cat "$out/.sfap_bins" 2>/dev/null)" = "$SFAP_BINS" ] &&
	   [ -z "$(find "$src" "$ROOT/gnss_crpa/src/gnss_aj_math.c" "$ROOT/gnss_crpa/src/gnss_aj_math.h" \
		-newer "$out/component.xml" -type f -not -path '*/build/*' 2>/dev/null)" ]; then
		ok "sfap_pi IP up to date ($SFAP_BINS bins): $out (FORCE_HLS=1 rebuilds it)"
		return 0
	fi
	xilinx_run 'command -v vitis_hls' >/dev/null 2>&1 ||
		die "vitis_hls not found via VIVADO_SETTINGS=$VIVADO_SETTINGS (install Vitis $REQUIRED_XILINX_VERSION)"
	info "Vitis HLS: sfap_pi, $SFAP_BINS bins -> $out (about 15-30 min; SFAP_SKIP_CSIM=1 skips the C simulation)"
	local log="$src/build/build_hls.log"
	mkdir -p "$src/build"
	rm -f "$out/.sfap_bins"
	if ! xilinx_run "cd '$src' && GNSS_CRPA_IP_REPO='$FW_DIR/hdl/library/antsdr_ip' SFAP_FW_DIR='$ROOT/gnss_crpa/src' SFAP_LOG2N='$SFAP_LOG2N' SFAP_SKIP_CSIM='${SFAP_SKIP_CSIM:-0}' vitis_hls -f build_hls.tcl" 2>&1 | tee "$log" ||
	   ! grep -q "SFAP_PI_HLS: PASS" "$log"; then
		die "sfap_pi HLS build failed (see $log)"
	fi
	echo "$SFAP_BINS" > "$out/.sfap_bins"
	# an older filter core of this folder (crpa1-5) must not stay in the IP catalog
	rm -rf "$FW_DIR/hdl/library/antsdr_ip/smi_pi"
	ok "sfap_pi IP ($SFAP_BINS bins): $out"
}

require_hls() {
	[ "$TARGET" = ant ] || return 0
	if [ -f "$PATCH_DIR/$TARGET/0003-hdl-stap-antijam.patch" ] && [ -d "$FW_DIR/hdl/library/gnss_passthrough" ]; then
		sfap_bins_apply
	fi
	[ -d "$FW_DIR/buildroot/package/gnss-crpa" ] && sync_crpa
	if [ -f "$PATCH_DIR/$TARGET/0003-hdl-stap-antijam.patch" ] &&
	   { [ ! -f "$FW_DIR/hdl/library/antsdr_ip/sfap_pi/component.xml" ] ||
	     [ "$(cat "$FW_DIR/hdl/library/antsdr_ip/sfap_pi/.sfap_bins" 2>/dev/null)" != "$SFAP_BINS" ]; }; then
		cmd_hls
	fi
}

require_ready() {
	check_target
	[ -f "$FW_DIR/Makefile" ] || die "sources not fetched yet; run './build.sh fetch'"
	grep -Eq "^SUPPORTED_TARGETS:=.*\\b$TARGET\\b" "$FW_DIR/Makefile" ||
		die "plutosdr-fw is not patched for TARGET=$TARGET; run './build.sh patch'"
	# Without Vivado the Makefile silently downloads ADI's *PlutoSDR* XSA,
	# which is the wrong hardware for an ANTSDR. Refuse that.
	xilinx_run 'vivado -version' >/dev/null 2>&1 ||
		die "Vivado not usable via VIVADO_SETTINGS=$VIVADO_SETTINGS; run './build.sh check'"
}

run_make() {
	# Make variables must come from the environment, never as VAR=value
	# arguments: ADI's hdl/projects/scripts/project-xilinx.mk turns make
	# command-line variables into a project sub-folder name, and the later
	# 'cp hdl/projects/$TARGET/$TARGET.sdk/system_top.xsa' then fails.
	local a
	for a in "$@"; do
		case "$a" in
			[A-Za-z_]*=*) die "pass '$a' as an environment variable (e.g. '$a ./build.sh ...'), not as a make argument" ;;
		esac
	done
	info "make $* (TARGET=$TARGET, VIVADO_SETTINGS=$VIVADO_SETTINGS)"
	(cd "$FW_DIR" && make "$@")
}

cmd_hdl() {
	require_ready
	require_hls
	run_make build/system_top.xsa
	ok "HW platform: $FW_DIR/build/system_top.xsa"
	ok "Vivado project: $FW_DIR/hdl/projects/$TARGET/$TARGET.xpr"
}

cmd_fsbl() {
	require_ready
	require_hls
	run_make build/sdk/fsbl/Release/fsbl.elf
	ok "FSBL: $FW_DIR/build/sdk/fsbl/Release/fsbl.elf"
	ok "Bitstream: $FW_DIR/build/system_top.bit"
}

cmd_firmware() {
	require_ready
	require_hls
	run_make
	cmd_outputs
}

cmd_sdimg() {
	require_ready
	local f
	for f in build/sdk/fsbl/Release/fsbl.elf build/system_top.bit build/u-boot.elf \
		linux/arch/arm/boot/uImage "build/zynq-$TARGET.dtb" build/uboot-env.txt \
		build/rootfs.cpio.gz; do
		[ -f "$FW_DIR/$f" ] || die "$f is missing; run './build.sh firmware' first"
	done
	# The Makefile does a plain 'mkdir build_sdimg', so a second run would fail.
	rm -rf "$FW_DIR/build_sdimg"
	run_make sdimg
	cmd_outputs
}

# --------------------------------------------------------------------------
# baremetal: design-A FPGA + the anti-jam firmware without Linux (ant only)
# --------------------------------------------------------------------------
cmd_baremetal() {
	require_ready
	[ "$TARGET" = ant ] || die "baremetal is for TARGET=ant (E310) only"
	require_hls
	local xsa="$FW_DIR/build/system_top.xsa" out="$ROOT/build_baremetal"
	local scripts="$ROOT/baremetal/Scripts" src="$ROOT/baremetal/app_gnss_e310"
	[ -d "$src" ] || die "missing $src"
	if [ ! -f "$xsa" ]; then
		info "No FPGA build yet: building the hardware first (Vivado)"
		run_make build/system_top.xsa
	fi
	mkdir -p "$out"
	(cd "$out" && unzip -o -q "$xsa" system_top.bit) || die "no system_top.bit in $xsa"

	# the firmware sources with this build's FFT size (gnss_sfap_cfg.h)
	rm -rf "$out/fw_src"
	cp -a "$src" "$out/fw_src"
	sfap_cfg_header "$out/fw_src"
	src="$out/fw_src"

	info "Bare-metal firmware (Vitis xsct, classic BSP, SD logging + band 7 on SD, SFAP-PI $SFAP_BINS bins)"
	xilinx_run "xsct '$scripts/build_software.tcl' '$xsa' '$out/vitis_ws' '$src' '$out/e310_gnss_app.elf' '${BM_DEFINES-GNSS_LOG_SD}'" \
		2>&1 | tee "$out/build_software.log"
	grep -q "SW_RESULT: PASS" "$out/build_software.log" || die "firmware build failed (see $out/build_software.log)"

	info "FSBL (Vitis xsct)"
	xilinx_run "xsct '$scripts/build_fsbl.tcl' '$xsa' '$out/fsbl_ws' '$out/fsbl.elf'" 2>&1 | tee "$out/build_fsbl.log"
	grep -q "FSBL_RESULT: PASS" "$out/build_fsbl.log" || die "FSBL build failed (see $out/build_fsbl.log)"

	info "BOOT.bin = FSBL + bitstream + firmware (bootgen)"
	printf 'the_ROM_image:\n{\n\t[bootloader] %s\n\t%s\n\t%s\n}\n' \
		"$out/fsbl.elf" "$out/system_top.bit" "$out/e310_gnss_app.elf" > "$out/boot.bif"
	rm -f "$out/BOOT.bin"
	xilinx_run "bootgen -arch zynq -image '$out/boot.bif' -o '$out/BOOT.bin' -w on" || die "bootgen failed"
	[ -f "$out/BOOT.bin" ] || die "bootgen produced no BOOT.bin"
	ok "Bare-metal boot image: $out/BOOT.bin ($(du -h "$out/BOOT.bin" | cut -f1))"
	ok "Copy ONLY BOOT.bin to the SD card (no uImage, devicetree, ramdisk or uEnv.txt)."
}

cmd_outputs() {
	local b="$FW_DIR/build" s="$FW_DIR/build_sdimg" f
	info "QSPI-flash firmware ($b)"
	for f in boot.bin boot.frm boot.dfu "$TARGET.itb" "$TARGET.frm" "$TARGET.dfu" \
		uboot-env.bin uboot-env.dfu system_top.bit system_top.xsa \
		sdk/fsbl/Release/fsbl.elf u-boot.elf zImage rootfs.cpio.gz "zynq-$TARGET.dtb"; do
		[ -f "$b/$f" ] && printf '   %-28s %s\n' "$f" "$(du -h "$b/$f" | cut -f1)"
	done
	if [ -d "$s" ]; then
		info "SD-card boot files ($s) - copy all of them to a FAT32 SD card"
		for f in "$s"/*; do
			printf '   %-28s %s\n' "$(basename "$f")" "$(du -h "$f" | cut -f1)"
		done
	fi
}

# --------------------------------------------------------------------------
# reset
# --------------------------------------------------------------------------
cmd_reset() {
	[ -e "$FW_DIR/.git" ] || die "nothing to reset: $FW_DIR does not exist"
	if [ "${1:-}" != "--yes" ]; then
		if [ -t 0 ]; then
			read -r -p "Discard ALL changes and build outputs under $FW_DIR? [y/N] " ans
			case "$ans" in y|Y|yes|YES) ;; *) die "aborted" ;; esac
		else
			die "non-interactive: use './build.sh reset --yes'"
		fi
	fi
	local entry path sha dir
	git -C "$FW_DIR" reset -q --hard "$PLUTOSDR_FW_REV"
	git -C "$FW_DIR" clean -qxdf
	ok "plutosdr-fw reset"
	for entry in "${SUBMODULES[@]}"; do
		read -r path sha <<<"$entry"
		dir="$FW_DIR/$path"
		[ -e "$dir/.git" ] || continue
		git -C "$dir" reset -q --hard "$sha"
		if [ "$path" = buildroot ]; then
			git -C "$dir" clean -qxdf -e /dl   # keep downloaded source tarballs
		else
			git -C "$dir" clean -qxdf
		fi
		ok "$path reset"
	done
}

# --------------------------------------------------------------------------
cmd="${1:-help}"
[ $# -gt 0 ] && shift
case "$cmd" in
	deps)     cmd_deps ;;
	check)    cmd_check ;;
	fetch)    cmd_fetch ;;
	patch)    cmd_patch ;;
	hls)      cmd_hls ;;
	hdl)      cmd_hdl ;;
	fsbl)     cmd_fsbl ;;
	firmware) cmd_firmware ;;
	sdimg)    cmd_sdimg ;;
	all)      cmd_check; cmd_fetch; cmd_patch; cmd_hls; cmd_firmware; cmd_sdimg ;;
	baremetal) cmd_baremetal ;;
	all-baremetal) cmd_check; cmd_fetch; cmd_patch; cmd_hls; cmd_baremetal ;;
	outputs)  cmd_outputs ;;
	reset)    cmd_reset "$@" ;;
	make)     require_ready; run_make "$@" ;;
	help|-h|--help) usage ;;
	*)        usage; die "unknown command '$cmd'" ;;
esac
