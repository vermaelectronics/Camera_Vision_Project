@echo off
setlocal EnableExtensions
REM ============================================================================
REM  Build_All.bat -- Command Prompt build driver for the ANTSDR E310 GNSS
REM  passthrough Vivado project.
REM
REM  Vivado/Scripts/create_project.tcl (see its own header) says it is meant to
REM  be driven by "Automation/PowerShell/Build_All.ps1" -- that script is
REM  PowerShell and is not part of this handoff. This is the cmd.exe / Command
REM  Prompt equivalent: it runs, IN ORDER, every Tcl script needed to
REM  reconstruct the Vivado project from source and, optionally, build a
REM  bitstream, targeting Vivado 2026.1.
REM
REM  WHAT IT RUNS, IN ORDER
REM    1. Vivado\Scripts\package_library_ip.tcl   x11  (once per Analog Devices
REM       library IP that Source\BlockDesign\system_bd.tcl instantiates by
REM       name -- axi_sysid, sysid_rom, axi_ad9361, util_tdd_sync, util_wfifo,
REM       util_pack\util_cpack2, axi_dmac, util_rfifo, util_pack\util_upack2,
REM       axi_gpreg, xilinx\util_clkdiv. The other cells the block design uses
REM       -- processing_system7, xlconcat, proc_sys_reset, util_vector_logic --
REM       ship in Vivado's own IP catalog and need no packaging step.)
REM    2. Source\IP\gnss_passthrough\gnss_passthrough_ip.tcl   x1  (packages
REM       this project's own custom IP)
REM    3. Vivado\Scripts\create_project.tcl   x1  (creates the .xpr and sources
REM       Source\BlockDesign\system_bd.tcl to build the block design)
REM    4. Vivado\Scripts\build.tcl   x1  (synthesis + implementation + write
REM       bitstream + write_hw_platform -- only when called with "build")
REM
REM  Vivado\Scripts\export_bd_to_source.tcl is NOT run here. Per its own
REM  header it is a one-way GUI-to-source sync tool an engineer runs by hand
REM  AFTER editing the block design in the Vivado GUI; it has nothing to do
REM  with building the project from source.
REM
REM  USAGE  (from an ordinary Command Prompt, cmd.exe -- not PowerShell)
REM    cd /d C:\path\to\this\project
REM    Automation\CommandPrompt\Build_All.bat
REM        Package every IP and (re)create the Vivado project + block design.
REM    Automation\CommandPrompt\Build_All.bat build
REM        Also run synthesis, implementation, write the bitstream and export
REM        the .xsa. This can take 15-60+ minutes.
REM    Automation\CommandPrompt\Build_All.bat clean
REM        Wipe the disposable vendor working copy and the packaged IP repo
REM        first, so every IP is repackaged from scratch instead of reused.
REM    Automation\CommandPrompt\Build_All.bat clean build
REM        Both of the above together.
REM
REM  ONE-TIME SETUP BEFORE THE FIRST RUN
REM    1. Install Vivado 2026.1.
REM    2. Place the Analog Devices HDL reference library at:
REM         (this project's root)\Vendor\ADI_hdl_2026_r1_update
REM       It is deliberately NOT part of this repository -- Vivado\Scripts\
REM       create_project.tcl treats it as an external, pristine, read-only
REM       tree (see that script's own header) that this build script copies
REM       into a disposable working copy before packaging anything, so
REM       Vendor\ itself is never written to. Obtain it from Analog Devices'
REM       "hdl" repository (https://github.com/analogdevicesinc/hdl) at the
REM       release matching this folder name, and lay it out so that
REM       Vendor\ADI_hdl_2026_r1_update\ is immediately followed by that
REM       repository's own top level: library, projects, scripts, and so on.
REM
REM  CONFIGURATION
REM    Edit the USER CONFIGURATION block below once for your machine, or set
REM    the same-named environment variable before calling this script -- a
REM    value already set in your environment is left alone and always wins
REM    over the default here.
REM ============================================================================

REM ---------------------------------------------------------------------------
REM  USER CONFIGURATION
REM ---------------------------------------------------------------------------

REM Directory that CONTAINS Vivado\bin\vivado.bat (matches Source\Config\
REM local.settings.template.json: "the directory that CONTAINS Vivado\bin\
REM vivado.bat"). Only used if vivado.bat is not already on PATH.
if not defined VIVADO_INSTALL_ROOT set "VIVADO_INSTALL_ROOT=C:\AMDDesignTools\2026.1"

REM Parallel jobs for synthesis/implementation. Used only by the "build" stage.
if not defined VIVADO_JOBS set "VIVADO_JOBS=%NUMBER_OF_PROCESSORS%"

REM Vivado version this project is pinned to. The upstream Analog Devices
REM scripts abort if the running Vivado does not match this exactly (see
REM Vendor\ADI_hdl_2026_r1_update\scripts\adi_env.tcl); this is how the check
REM is told 2026.1 is expected here instead of the vendor tree's own default.
if not defined REQUIRED_VIVADO_VERSION set "REQUIRED_VIVADO_VERSION=2026.1"

REM ---------------------------------------------------------------------------
REM  FIXED LAYOUT -- derived from this script's own location. Not meant to be
REM  edited; this script must stay at (project root)\Automation\CommandPrompt\.
REM ---------------------------------------------------------------------------
set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..\..") do set "ROOT=%%~fI"

set "SOURCE_DIR=%ROOT%\Source"
set "VIVADO_SCRIPTS=%ROOT%\Vivado\Scripts"
set "VIVADO_PROJECT_DIR=%ROOT%\Vivado\Project"
set "VENDOR_HDL_SRC=%ROOT%\Vendor\ADI_hdl_2026_r1_update"
set "WORK=%ROOT%\Build\VendorWork"

REM Required by Vivado\Scripts\create_project.tcl and Source\BlockDesign\
REM system_bd.tcl -- see their own headers for exactly how each is used.
set "GNSS_CRPA_ROOT=%ROOT%"
set "ADI_HDL_DIR=%WORK%\hdl"
set "GNSS_CRPA_IP_REPO=%ROOT%\Build\ip_repo"

set "XPR=%VIVADO_PROJECT_DIR%\antsdr_e310_gnss.xpr"
set "BIT_OUT=%ROOT%\Build\Output\antsdr_e310_gnss.bit"
set "XSA_OUT=%ROOT%\Build\Output\antsdr_e310_gnss.xsa"

set "DO_CLEAN=0"
set "DO_BUILD=0"
for %%A in (%*) do if /I "%%A"=="clean" set "DO_CLEAN=1"
for %%A in (%*) do if /I "%%A"=="build" set "DO_BUILD=1"

echo ============================================================================
echo  Build_All.bat  --  ROOT=%ROOT%
echo ============================================================================

REM ---------------------------------------------------------------------------
REM  STEP 0: find vivado.bat
REM ---------------------------------------------------------------------------
if not defined VIVADO_BAT (
  for /f "usebackq delims=" %%V in (`where vivado.bat 2^>nul`) do (
    if not defined VIVADO_BAT set "VIVADO_BAT=%%V"
  )
)
if not defined VIVADO_BAT (
  if exist "%VIVADO_INSTALL_ROOT%\Vivado\bin\vivado.bat" set "VIVADO_BAT=%VIVADO_INSTALL_ROOT%\Vivado\bin\vivado.bat"
)
if not exist "%VIVADO_BAT%" (
  echo ERROR: vivado.bat not found.
  echo   Looked on PATH, then at "%VIVADO_INSTALL_ROOT%\Vivado\bin\vivado.bat".
  echo   Either add Vivado to PATH, or set VIVADO_INSTALL_ROOT ^(or VIVADO_BAT
  echo   directly^) before running this script.
  goto :fail
)
echo Using Vivado: %VIVADO_BAT%
"%VIVADO_BAT%" -version

REM ---------------------------------------------------------------------------
REM  Sanity checks -- fail fast with a clear reason rather than deep inside a
REM  Tcl error, matching how the project's own scripts prefer to fail.
REM ---------------------------------------------------------------------------
if not exist "%VENDOR_HDL_SRC%\scripts\adi_env.tcl" (
  echo ERROR: Analog Devices HDL library not found at:
  echo   %VENDOR_HDL_SRC%
  echo   See the ONE-TIME SETUP note at the top of this script.
  goto :fail
)
if not exist "%SOURCE_DIR%\BlockDesign\system_bd.tcl" (
  echo ERROR: %SOURCE_DIR%\BlockDesign\system_bd.tcl not found.
  echo   ROOT resolved to: %ROOT%  -- is this script still under Automation\CommandPrompt\?
  goto :fail
)

if "%DO_CLEAN%"=="1" (
  echo.
  echo === clean: removing disposable working copy and packaged IP repo ===
  if exist "%WORK%" rmdir /s /q "%WORK%"
  if exist "%GNSS_CRPA_IP_REPO%" rmdir /s /q "%GNSS_CRPA_IP_REPO%"
)

REM ---------------------------------------------------------------------------
REM  STEP 1: disposable working copy of the vendor library.
REM
REM  create_project.tcl requires ADI_HDL_DIR to point at a copy it is allowed
REM  to write packaged IP into, never at Vendor\ itself (which stays pristine
REM  and read-only). robocopy only touches files that are new or changed, so
REM  running this every time is cheap once the working copy already exists.
REM ---------------------------------------------------------------------------
echo.
echo === Step 1: syncing vendor library into the disposable working copy ===
echo   %VENDOR_HDL_SRC%
echo   -^>  %ADI_HDL_DIR%
robocopy "%VENDOR_HDL_SRC%" "%ADI_HDL_DIR%" /E /NFL /NDL /NJH /NJS /NC /NS /NP
if errorlevel 8 (
  echo ERROR: robocopy failed while copying the vendor library.
  goto :fail
)

REM ---------------------------------------------------------------------------
REM  STEP 2: package every Analog Devices library IP the block design needs.
REM  See the :PackageLib subroutine at the bottom of this file.
REM ---------------------------------------------------------------------------
echo.
echo === Step 2: packaging Analog Devices library IP ===

call :PackageLib "axi_sysid"
if errorlevel 1 goto :fail
call :PackageLib "sysid_rom"
if errorlevel 1 goto :fail
call :PackageLib "axi_ad9361"
if errorlevel 1 goto :fail
call :PackageLib "util_tdd_sync"
if errorlevel 1 goto :fail
call :PackageLib "util_wfifo"
if errorlevel 1 goto :fail
call :PackageLib "util_pack\util_cpack2"
if errorlevel 1 goto :fail
call :PackageLib "axi_dmac"
if errorlevel 1 goto :fail
call :PackageLib "util_rfifo"
if errorlevel 1 goto :fail
call :PackageLib "util_pack\util_upack2"
if errorlevel 1 goto :fail
call :PackageLib "axi_gpreg"
if errorlevel 1 goto :fail
call :PackageLib "xilinx\util_clkdiv"
if errorlevel 1 goto :fail

REM ---------------------------------------------------------------------------
REM  STEP 3: package this project's own custom IP.
REM  Always re-run: gnss_passthrough_ip.tcl deletes and rebuilds its output
REM  directory every time so it can never go stale against edited RTL.
REM ---------------------------------------------------------------------------
echo.
echo === Step 3: packaging custom IP: gnss_passthrough ===
"%VIVADO_BAT%" -mode batch -source "%SOURCE_DIR%\IP\gnss_passthrough\gnss_passthrough_ip.tcl" -tclargs "%SOURCE_DIR%" "%GNSS_CRPA_IP_REPO%"
if errorlevel 1 (
  echo ERROR: gnss_passthrough IP packaging failed.
  goto :fail
)

REM ---------------------------------------------------------------------------
REM  STEP 4: create the Vivado project and build the block design.
REM  create_project.tcl expects the caller to already be cd'd into
REM  Vivado\Project (see its own header); it uses create_project ... -force,
REM  so re-running this against an existing project overwrites it in place.
REM ---------------------------------------------------------------------------
echo.
echo === Step 4: creating Vivado project + block design ===
if not exist "%VIVADO_PROJECT_DIR%" mkdir "%VIVADO_PROJECT_DIR%"
pushd "%VIVADO_PROJECT_DIR%"
"%VIVADO_BAT%" -mode batch -source "%VIVADO_SCRIPTS%\create_project.tcl"
if errorlevel 1 (
  popd
  echo ERROR: create_project.tcl failed.
  goto :fail
)
popd

REM ---------------------------------------------------------------------------
REM  STEP 5 (optional): synthesis, implementation, bitstream, .xsa.
REM  Only runs when this script is called with "build" -- see build.tcl's own
REM  header for why it is kept separate from project creation.
REM ---------------------------------------------------------------------------
if "%DO_BUILD%"=="1" (
  echo.
  echo === Step 5: synthesis + implementation + bitstream + XSA ===
  echo   jobs=%VIVADO_JOBS%  -- this can take 15-60+ minutes
  "%VIVADO_BAT%" -mode batch -source "%VIVADO_SCRIPTS%\build.tcl" -tclargs "%XPR%" "%BIT_OUT%" "%XSA_OUT%" %VIVADO_JOBS%
  if errorlevel 1 (
    echo ERROR: build.tcl failed.
    goto :fail
  )
)

echo.
echo ============================================================================
echo  BUILD_ALL_RESULT: PASS
echo    Project : %XPR%
if "%DO_BUILD%"=="1" echo    Bitstream: %BIT_OUT%
if "%DO_BUILD%"=="1" echo    XSA      : %XSA_OUT%
if "%DO_BUILD%"=="0" echo    ^(bitstream not requested -- rerun with "build" to synthesize + implement^)
echo ============================================================================
exit /b 0

:fail
echo.
echo ============================================================================
echo  BUILD_ALL_RESULT: FAIL -- see the output above for the exact cause
echo ============================================================================
exit /b 1

REM ============================================================================
REM  Subroutines -- only reached via "call", never by falling through.
REM ============================================================================

:PackageLib
REM Usage: call :PackageLib  followed by one library's path relative to hdl\library
set "LIB_REL=%~1"
set "LIB_DIR=%ADI_HDL_DIR%\library\%LIB_REL%"
if not exist "%LIB_DIR%" (
  echo ERROR: library folder not found: "%LIB_DIR%"
  exit /b 2
)
if "%DO_CLEAN%"=="1" goto :PackageLib_run
if exist "%LIB_DIR%\component.xml" (
  echo   [skip, already packaged] %LIB_REL%
  exit /b 0
)
:PackageLib_run
echo   packaging %LIB_REL% ...
pushd "%LIB_DIR%"
"%VIVADO_BAT%" -mode batch -source "%VIVADO_SCRIPTS%\package_library_ip.tcl" -tclargs "%LIB_REL%"
if errorlevel 1 (
  popd
  echo ERROR: packaging failed for %LIB_REL%
  exit /b 2
)
popd
exit /b 0
