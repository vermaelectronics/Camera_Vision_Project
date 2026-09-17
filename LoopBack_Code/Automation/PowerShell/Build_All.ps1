# ============================================================================
#  Build_All.ps1  --  complete, non-interactive, command-line-only build
#
#  Runs every stage needed to turn Source/ + Vendor/ into a Vivado bitstream,
#  with no GUI interaction at any point:
#
#    Stage 1  Resolve the Vivado toolchain and load/create per-machine settings
#    Stage 2  Copy the ADI HDL vendor tree into a disposable working copy
#             (Vendor/ itself is never written to)
#    Stage 3  Package the ADI library IP this design uses, plus the custom
#             gnss_passthrough IP, into a Vivado IP repository
#    Stage 4  Reconstruct the Vivado project from Source/ (create_project.tcl)
#    Stage 5  Synthesise, implement and write the bitstream (build.tcl)
#
#  Every Vivado invocation runs "-mode batch": nothing here ever opens the
#  Vivado GUI. See ../../README.md for the exact commands to run this file.
# ============================================================================

[CmdletBinding()]
param(
  # Resume from this stage instead of running everything. Stage numbers match
  # the list above (e.g. -FromStage 3 re-packages IP and re-runs create+build
  # without re-copying the vendor tree; -FromStage 5 just re-runs the build
  # against the Vivado project that already exists).
  [ValidateRange(1, 5)]
  [int]$FromStage = 1,

  # Parallel jobs for synthesis/implementation. Defaults to
  # Source/Config/local.settings.json -> build.jobs, or 4 if that is also unset.
  [int]$Jobs,

  # Force a fresh copy of the vendor HDL tree into Build/VendorWork even if one
  # already exists there.
  [switch]$CleanVendorWork
)

$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
Set-StrictMode -Version Latest

# ----------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------

function Write-Stage {
  param([int]$Number, [string]$Text)
  Write-Host ""
  Write-Host "=== Stage $Number/5: $Text ===" -ForegroundColor Magenta
}

function Get-LocalSettings {
  param([Parameter(Mandatory)][string]$Root)

  $templatePath = Join-Path $Root 'Source/Config/local.settings.template.json'
  $settingsPath = Join-Path $Root 'Source/Config/local.settings.json'

  if (-not (Test-Path $settingsPath)) {
    if (-not (Test-Path $templatePath)) {
      throw "Neither local.settings.json nor local.settings.template.json exist under Source/Config."
    }
    Copy-Item -Path $templatePath -Destination $settingsPath
    Write-Host "Created $settingsPath from the template (edit it if auto-detection below fails)." -ForegroundColor Yellow
  }

  try {
    return Get-Content -Raw -Path $settingsPath | ConvertFrom-Json
  } catch {
    throw "Failed to parse $settingsPath as JSON: $_"
  }
}

# Finds a Vivado executable without ever guessing silently: every fallback is
# an explicit, logged step, and the final failure says exactly what to set.
function Resolve-VivadoExecutable {
  param($Settings)

  $installRoot = $Settings.toolchain.install_root
  if ($installRoot) {
    $exeName = if ($IsWindows) { 'vivado.bat' } else { 'vivado' }
    $candidate = Join-Path (Join-Path $installRoot 'Vivado/bin') $exeName
    if (Test-Path $candidate) {
      Write-Host "  Using toolchain.install_root from local.settings.json"
      return (Resolve-Path $candidate).Path
    }
    Write-Host "  WARNING: toolchain.install_root is set to '$installRoot' but $candidate does not exist; falling back to auto-detection." -ForegroundColor Yellow
  }

  $onPath = Get-Command 'vivado' -ErrorAction SilentlyContinue
  if ($onPath) {
    Write-Host "  Found vivado on PATH"
    return $onPath.Source
  }

  $searchSpecs = @()
  if ($IsWindows) {
    $searchSpecs += @{ Roots = @('C:\Xilinx', 'D:\Xilinx');                Sub = 'Vivado\*\bin\vivado.bat' }
    $searchSpecs += @{ Roots = @('C:\AMDDesignTools', 'D:\AMDDesignTools'); Sub = '*\Vivado\bin\vivado.bat' }
  } else {
    $searchSpecs += @{ Roots = @('/opt/Xilinx', '/tools/Xilinx');   Sub = 'Vivado/*/bin/vivado' }
    $searchSpecs += @{ Roots = @('/opt/AMDDesignTools');            Sub = '*/Vivado/bin/vivado' }
  }

  $found = @()
  foreach ($spec in $searchSpecs) {
    foreach ($root in $spec.Roots) {
      if (Test-Path $root) {
        $found += Get-ChildItem -Path (Join-Path $root $spec.Sub) -ErrorAction SilentlyContinue
      }
    }
  }
  if ($found.Count -gt 0) {
    $best = $found | Sort-Object FullName -Descending | Select-Object -First 1
    Write-Host "  Found vivado under a standard install root: $($best.FullName)"
    return $best.FullName
  }

  if ($IsWindows) {
    $shortcutDirs = @(
      (Join-Path $env:ProgramData 'Microsoft\Windows\Start Menu\Programs'),
      (Join-Path $env:AppData 'Microsoft\Windows\Start Menu\Programs')
    )
    $shell = $null
    foreach ($dir in $shortcutDirs) {
      if (-not (Test-Path $dir)) { continue }
      $links = Get-ChildItem -Path $dir -Filter '*Vivado*.lnk' -Recurse -ErrorAction SilentlyContinue
      foreach ($link in $links) {
        try {
          if (-not $shell) { $shell = New-Object -ComObject WScript.Shell }
          $target = $shell.CreateShortcut($link.FullName).TargetPath
          if ($target -and (Test-Path $target) -and ($target -match 'vivado\.(bat|exe)$')) {
            Write-Host "  Found vivado via Start Menu shortcut: $($link.Name)"
            return $target
          }
        } catch {
          # Unreadable or non-application shortcut; not a candidate.
        }
      }
    }
  }

  throw "Could not find a Vivado executable anywhere (local.settings.json, PATH, standard install roots, Start Menu). Set toolchain.install_root in Source/Config/local.settings.json, or add vivado to PATH."
}

function Get-VivadoVersionString {
  param([Parameter(Mandatory)][string]$VivadoExe)

  $pattern = 'Vivado\s+v(\d+\.\d+(?:\.\d+)?)'
  $out = & $VivadoExe -version 2>&1
  foreach ($candidateLine in $out) {
    if ($candidateLine -match $pattern) {
      return $Matches[1]
    }
  }
  throw "Could not determine the Vivado version from '$VivadoExe -version'. Output was:`n$($out -join [Environment]::NewLine)"
}

# Runs one Tcl script under "vivado -mode batch", tees its output to a log
# file, and only calls it a pass if the exit code is 0 AND the script's own
# success marker appears in the log. Never trusts exit code alone (build.tcl's
# own success criteria comment applies here too).
function Invoke-VivadoTcl {
  param(
    [Parameter(Mandatory)][string]$VivadoExe,
    [Parameter(Mandatory)][string]$TclScript,
    [string[]]$TclArgs = @(),
    [Parameter(Mandatory)][string]$WorkingDirectory,
    [Parameter(Mandatory)][string]$LogFile,
    [Parameter(Mandatory)][string]$SuccessPattern,
    [string]$StepName = (Split-Path $TclScript -Leaf)
  )

  $logDir = Split-Path $LogFile -Parent
  if ($logDir -and -not (Test-Path $logDir)) { New-Item -ItemType Directory -Force -Path $logDir | Out-Null }
  if (-not (Test-Path $WorkingDirectory)) { New-Item -ItemType Directory -Force -Path $WorkingDirectory | Out-Null }

  $argList = @('-mode', 'batch', '-source', $TclScript)
  if ($TclArgs.Count -gt 0) {
    $argList += '-tclargs'
    $argList += $TclArgs
  }

  Write-Host "  -> $StepName" -ForegroundColor Cyan
  Write-Host "     vivado $($argList -join ' ')" -ForegroundColor DarkGray
  Write-Host "     cwd: $WorkingDirectory" -ForegroundColor DarkGray

  Push-Location $WorkingDirectory
  try {
    # Piped through Write-Host (not left as pipeline output) so this statement
    # cannot leak vivado's chatter into the function's own return value -
    # every caller relies on that return value being just the log text.
    & $VivadoExe @argList 2>&1 | Tee-Object -FilePath $LogFile | ForEach-Object { Write-Host $_ }
    $exitCode = $LASTEXITCODE
  } finally {
    Pop-Location
  }

  $log = Get-Content -Raw -Path $LogFile -ErrorAction SilentlyContinue
  $ok = ($exitCode -eq 0) -and ($null -ne $log) -and ($log -match $SuccessPattern)

  if (-not $ok) {
    Write-Host "----- last 40 lines of $LogFile -----" -ForegroundColor Red
    Get-Content -Path $LogFile -Tail 40 -ErrorAction SilentlyContinue | ForEach-Object { Write-Host $_ -ForegroundColor Red }
    throw "$StepName FAILED (vivado exit code $exitCode). Full log: $LogFile"
  }

  Write-Host "     OK" -ForegroundColor Green
  return $log
}

# ----------------------------------------------------------------------------
#  Stage 1: toolchain + environment (always runs; every later stage needs it,
#  and -FromStage's minimum value is 1 so this is never itself skipped)
# ----------------------------------------------------------------------------

$scriptStart = Get-Date
$Root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path

Write-Host "GNSS-CRPA / ANTSDR E310 V1 -- command-line Vivado build"
Write-Host "Project root: $Root"

Write-Stage 1 "Toolchain and environment"
$settings = Get-LocalSettings -Root $Root
$vivadoExe = Resolve-VivadoExecutable -Settings $settings
Write-Host "  Vivado executable       : $vivadoExe"

$detectedVersion = Get-VivadoVersionString -VivadoExe $vivadoExe
$requiredVersion = if ($settings.build.force_required_version) { $settings.build.force_required_version } else { $detectedVersion }
Write-Host "  Detected Vivado version : $detectedVersion"
Write-Host "  REQUIRED_VIVADO_VERSION : $requiredVersion"

if (-not $PSBoundParameters.ContainsKey('Jobs')) {
  $Jobs = if ($settings.build.jobs) { [int]$settings.build.jobs } else { 4 }
}
Write-Host "  Parallel jobs           : $Jobs"

$BuildDir      = Join-Path $Root 'Build'
$VendorWorkHdl = Join-Path $BuildDir 'VendorWork/hdl'
$IpRepoDir     = Join-Path $BuildDir 'ip_repo'
$BitstreamDir  = Join-Path $BuildDir 'Bitstream'
$LogDir        = Join-Path $BuildDir 'Logs'
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

$env:GNSS_CRPA_ROOT          = $Root
$env:ADI_HDL_DIR             = $VendorWorkHdl
$env:GNSS_CRPA_IP_REPO       = $IpRepoDir
$env:REQUIRED_VIVADO_VERSION = $requiredVersion

# The 12 ADI library cores this design's block design instantiates (mirrors
# LIB_DEPS in Vendor/MicroPhase_E310_V1/hdl/projects/antsdre310/Makefile,
# the upstream reference design this project is forked from).
$libDeps = @(
  'axi_ad9361',
  'axi_dmac',
  'axi_gpreg',
  'axi_sysid',
  'sysid_rom',
  'util_pack/util_cpack2',
  'util_pack/util_upack2',
  'util_rfifo',
  'util_tdd_sync',
  'util_wfifo',
  'xilinx/axi_xcvrlb',
  'xilinx/util_clkdiv'
)

try {

  # --------------------------------------------------------------------------
  #  Stage 2: disposable vendor HDL working copy
  # --------------------------------------------------------------------------
  if ($FromStage -le 2) {
    Write-Stage 2 "Preparing a disposable copy of the ADI HDL vendor tree"
    # MicroPhase_E310_V1/hdl, not ADI_hdl_2026_r1_update: this is the exact
    # tree system_bd.tcl's MOD-0/MOD-1/MOD-2 changes were diffed against, and
    # it is the one whose adi_board.tcl correctly instantiates "xlconstant"
    # for GND/VCC ties. ADI_hdl_2026_r1_update's adi_board.tcl instead calls
    # "ad_ip_instance ilconstant", which is not a real IP anywhere in that
    # tree or in Vivado's own catalog, and fails create_project.tcl the first
    # time a block design connects a pin to a literal GND/VCC.
    $adiSource = Join-Path $Root 'Vendor/MicroPhase_E310_V1/hdl'
    if (-not (Test-Path $adiSource)) {
      throw "Vendor tree not found: $adiSource"
    }

    if ($CleanVendorWork -and (Test-Path $VendorWorkHdl)) {
      Write-Host "  -CleanVendorWork given: removing $VendorWorkHdl"
      Remove-Item -Path $VendorWorkHdl -Recurse -Force
    }

    if (Test-Path (Join-Path $VendorWorkHdl 'library')) {
      Write-Host "  Already present, skipping copy: $VendorWorkHdl"
      Write-Host "  (pass -CleanVendorWork to force a fresh copy)"
    } else {
      foreach ($sub in @('library', 'scripts', 'projects/scripts')) {
        $srcSub = Join-Path $adiSource $sub
        if (-not (Test-Path $srcSub)) {
          if ($sub -eq 'scripts') {
            # MicroPhase_E310_V1/hdl (unlike newer ADI HDL layouts) has no
            # top-level scripts/adi_env.tcl - only library/scripts/ and
            # projects/scripts/ copies. create_project.tcl already falls back
            # to projects/scripts/adi_env.tcl when this is absent, so skip
            # rather than fail.
            Write-Host "  (no top-level $sub/ in this vendor tree - OK, projects/scripts/adi_env.tcl covers it)"
            continue
          }
          throw "Expected vendor subtree missing: $srcSub"
        }
        $dstSub = Join-Path $VendorWorkHdl $sub
        $dstParent = Split-Path $dstSub -Parent
        New-Item -ItemType Directory -Force -Path $dstParent | Out-Null
        Write-Host "  Copying $sub ..."
        Copy-Item -Path $srcSub -Destination $dstSub -Recurse -Force
      }

      # MicroPhase_E310_V1/hdl predates auto_timing_fix_xilinx.tcl - it is a
      # newer ADI addition (this project's own note: "a workaround for Vivado
      # 2024.x/2025.x hold timing issues") that create_project.tcl requires
      # unconditionally for its AD9361/Zynq-7000 timing-closure aids. It is
      # self-contained (no further sources), so overlay it from the newer
      # tree rather than losing that timing-closure help by skipping it.
      $timingFixName = 'auto_timing_fix_xilinx.tcl'
      $timingFixDst = Join-Path $VendorWorkHdl "projects/scripts/$timingFixName"
      if (-not (Test-Path $timingFixDst)) {
        $timingFixSrc = Join-Path $Root "Vendor/ADI_hdl_2026_r1_update/projects/scripts/$timingFixName"
        if (Test-Path $timingFixSrc) {
          Write-Host "  Overlaying $timingFixName from Vendor/ADI_hdl_2026_r1_update (absent from MicroPhase_E310_V1/hdl)"
          Copy-Item -Path $timingFixSrc -Destination $timingFixDst -Force
        } else {
          throw "Neither vendor tree has $timingFixName; create_project.tcl requires it. Looked in: $timingFixSrc"
        }
      }

      Write-Host "  Vendor working copy ready: $VendorWorkHdl"
    }
  } else {
    Write-Stage 2 "Skipped (resuming from stage $FromStage)"
    if (-not (Test-Path (Join-Path $VendorWorkHdl 'library'))) {
      throw "Cannot resume at stage ${FromStage}: $VendorWorkHdl does not exist yet. Run from stage 2 or earlier first."
    }
  }

  # --------------------------------------------------------------------------
  #  Stage 3: package IP (ADI library cores + the custom gnss_passthrough IP)
  # --------------------------------------------------------------------------
  if ($FromStage -le 3) {
    Write-Stage 3 "Packaging IP ($($libDeps.Count) library cores + gnss_passthrough)"
    $packageScript = Join-Path $Root 'Vivado/Scripts/package_library_ip.tcl'

    foreach ($dep in $libDeps) {
      $libWorkDir = Join-Path $VendorWorkHdl "library/$dep"
      $logName = "03_ip_{0}.log" -f ($dep -replace '[\\/]', '_')
      Invoke-VivadoTcl -VivadoExe $vivadoExe -TclScript $packageScript -TclArgs @($dep) `
        -WorkingDirectory $libWorkDir -LogFile (Join-Path $LogDir $logName) `
        -SuccessPattern '(?m)^LIBIP_RESULT: PASS\s*$' -StepName "package library IP: $dep" | Out-Null
    }

    $gnssIpScript = Join-Path $Root 'Source/IP/gnss_passthrough/gnss_passthrough_ip.tcl'
    $sourceDir = Join-Path $Root 'Source'
    Invoke-VivadoTcl -VivadoExe $vivadoExe -TclScript $gnssIpScript -TclArgs @($sourceDir, $IpRepoDir) `
      -WorkingDirectory $BuildDir -LogFile (Join-Path $LogDir '03_ip_gnss_passthrough.log') `
      -SuccessPattern '(?m)^IP_PACKAGE_OK: ' -StepName 'package custom IP: gnss_passthrough' | Out-Null
  } else {
    Write-Stage 3 "Skipped (resuming from stage $FromStage)"
    $gnssComponent = Join-Path $IpRepoDir 'gnss_passthrough/component.xml'
    if (-not (Test-Path $gnssComponent)) {
      throw "Cannot resume at stage ${FromStage}: $gnssComponent does not exist yet. Run from stage 3 or earlier first."
    }
  }

  # --------------------------------------------------------------------------
  #  Stage 4: create the Vivado project from Source/
  # --------------------------------------------------------------------------
  if ($FromStage -le 4) {
    Write-Stage 4 "Creating the Vivado project"
    $projectDir = Join-Path $Root 'Vivado/Project'
    New-Item -ItemType Directory -Force -Path $projectDir | Out-Null
    $createScript = Join-Path $Root 'Vivado/Scripts/create_project.tcl'
    Invoke-VivadoTcl -VivadoExe $vivadoExe -TclScript $createScript `
      -WorkingDirectory $projectDir -LogFile (Join-Path $LogDir '04_create_project.log') `
      -SuccessPattern '(?m)^CREATE_RESULT: PASS\s*$' -StepName 'create Vivado project' | Out-Null
  } else {
    Write-Stage 4 "Skipped (resuming from stage $FromStage)"
  }

  # --------------------------------------------------------------------------
  #  Stage 5: synthesis, implementation, bitstream + hardware handoff (XSA)
  # --------------------------------------------------------------------------
  Write-Stage 5 "Synthesis, implementation, and writing the bitstream"
  $xpr = Join-Path $Root 'Vivado/Project/antsdr_e310_gnss.xpr'
  if (-not (Test-Path $xpr)) {
    throw "Project file not found: $xpr. Run with -FromStage 4 or lower first to create it."
  }
  New-Item -ItemType Directory -Force -Path $BitstreamDir | Out-Null
  $bitOut = Join-Path $BitstreamDir 'antsdr_e310_gnss.bit'
  $xsaOut = Join-Path $BitstreamDir 'antsdr_e310_gnss.xsa'
  $buildScript = Join-Path $Root 'Vivado/Scripts/build.tcl'
  Invoke-VivadoTcl -VivadoExe $vivadoExe -TclScript $buildScript `
    -TclArgs @($xpr, $bitOut, $xsaOut, $Jobs) `
    -WorkingDirectory $BitstreamDir -LogFile (Join-Path $LogDir '05_build.log') `
    -SuccessPattern '(?m)^BUILD_RESULT: PASS\s*$' -StepName 'synthesize, implement, write bitstream' | Out-Null

  $elapsed = (Get-Date) - $scriptStart
  Write-Host ""
  Write-Host "Bitstream               : $bitOut" -ForegroundColor Green
  Write-Host "Hardware platform (XSA) : $xsaOut" -ForegroundColor Green
  Write-Host "BUILD_ALL_RESULT: PASS (elapsed $($elapsed.ToString('hh\:mm\:ss')))" -ForegroundColor Green
  exit 0

} catch {
  Write-Host ""
  Write-Host "BUILD_ALL_RESULT: FAIL" -ForegroundColor Red
  Write-Host $_.Exception.Message -ForegroundColor Red
  exit 1
}
