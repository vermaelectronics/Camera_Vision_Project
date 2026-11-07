#!/usr/bin/env bash
#
# ANTSDR firmware build helper - Vivado/Vitis 2023.2, ADI plutosdr-fw v0.39
#
# Default target is the ANTSDR E310 (TARGET=ant). E200 / E310v2 work the same
# way with TARGET=e200 / TARGET=e310v2.
#
#   ./build.sh deps       install Ubuntu build packages (uses sudo apt-get)
#   ./build.sh check      check host tools and Vivado/Vitis 2023.2
#   ./build.sh fetch      clone plutosdr-fw v0.39 + submodules at pinned commits
#   ./build.sh patch      apply the ANTSDR patches (safe to re-run)
#   ./build.sh hdl        Vivado only: build/system_top.xsa (+ bitstream)
#   ./build.sh fsbl       Vitis/xsct: build/sdk/fsbl/Release/fsbl.elf
#   ./build.sh firmware   full firmware: boot.bin, ant.frm/.dfu, boot.frm/.dfu ...
#   ./build.sh sdimg      SD-card image: build_sdimg/BOOT.bin + uImage + ...
#   ./build.sh all        check + fetch + patch + firmware + sdimg
#   ./build.sh outputs    list the generated files
#   ./build.sh reset      restore pristine v0.39 sources (drops patches and builds)
#   ./build.sh make ARGS  run plutosdr-fw's make with the right environment
#
# Environment overrides:
#   TARGET           ant (E310, default) | e200 | e310v2
#   XILINX_DIR       Xilinx install root (auto: /tools/Xilinx, /opt/Xilinx, ~/Xilinx)
#   VIVADO_SETTINGS  settings script sourced for Xilinx tools (default: ./xilinx_env.sh)
#   SKIP_LEGAL=1     skip buildroot legal-info (faster; no legal-info tarball)

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW_DIR="$ROOT/plutosdr-fw"
PATCH_DIR="$ROOT/patch"

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
	for t in xsct bootgen; do
		local p
		p="$(xilinx_run "command -v $t" || true)"
		if [ -n "$p" ]; then
			ok "$t: $p"
		else
			die "$t not found after sourcing $VIVADO_SETTINGS - install Vitis $REQUIRED_XILINX_VERSION (it provides xsct for the FSBL)"
		fi
	done

	info "Shell environment"
	if command -v vivado >/dev/null 2>&1 || command -v xsct >/dev/null 2>&1; then
		warn "Xilinx tools are already on PATH in this shell. Use a fresh terminal without"
		warn "sourcing settings64.sh: buildroot can break with Xilinx tools on PATH."
	else
		ok "Xilinx tools not on PATH (good: the Makefile sources them when needed)"
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
apply_patch() {
	local dir="$1" patch="$2" name
	name="${patch#"$PATCH_DIR"/}"
	[ -f "$patch" ] || die "missing patch file $patch"
	if git -C "$dir" apply --check "$patch" 2>/dev/null; then
		git -C "$dir" apply --whitespace=nowarn "$patch"
		ok "applied         $name"
	elif git -C "$dir" apply --reverse --check "$patch" 2>/dev/null; then
		ok "already applied $name"
	else
		git -C "$dir" apply --check "$patch" || true
		die "$name does not apply to ${dir#"$ROOT"/} (modified sources or another TARGET was patched); run './build.sh reset'"
	fi
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
	apply_patch "$FW_DIR/hdl"         "$PATCH_DIR/0001-add-three-ant-hdl-support-bump-to-v0.39-v1.patch"
	apply_patch "$FW_DIR/u-boot-xlnx" "$PATCH_DIR/$TARGET/0001-add-support-uboot.patch"
	apply_patch "$FW_DIR/linux"       "$PATCH_DIR/$TARGET/0001-add-support-linux.patch"
	apply_patch "$FW_DIR/buildroot"   "$PATCH_DIR/$TARGET/0001-add-support-buildroot.patch"
	apply_patch "$FW_DIR"             "$PATCH_DIR/$TARGET/0001-add-support-makefile-scripts.patch"
}

# --------------------------------------------------------------------------
# make wrappers
# --------------------------------------------------------------------------
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
	run_make build/system_top.xsa
	ok "HW platform: $FW_DIR/build/system_top.xsa"
	ok "Vivado project: $FW_DIR/hdl/projects/$TARGET/$TARGET.xpr"
}

cmd_fsbl() {
	require_ready
	run_make build/sdk/fsbl/Release/fsbl.elf
	ok "FSBL: $FW_DIR/build/sdk/fsbl/Release/fsbl.elf"
	ok "Bitstream: $FW_DIR/build/system_top.bit"
}

cmd_firmware() {
	require_ready
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
	hdl)      cmd_hdl ;;
	fsbl)     cmd_fsbl ;;
	firmware) cmd_firmware ;;
	sdimg)    cmd_sdimg ;;
	all)      cmd_check; cmd_fetch; cmd_patch; cmd_firmware; cmd_sdimg ;;
	outputs)  cmd_outputs ;;
	reset)    cmd_reset "$@" ;;
	make)     require_ready; run_make "$@" ;;
	help|-h|--help) usage ;;
	*)        usage; die "unknown command '$cmd'" ;;
esac
