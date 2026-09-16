@echo off
rem Complete, non-interactive Vivado bitstream build - Windows Command Prompt entry point.
rem Everything happens from the command line; no Vivado GUI is ever opened.
rem Usage:
rem   build.bat                    full build, produces Build\Bitstream\antsdr_e310_gnss.bit
rem   build.bat -FromStage 5       re-run just the synth/impl/bitstream stage
rem   build.bat -CleanVendorWork   force a fresh copy of the vendor HDL tree
setlocal
set "SCRIPT_DIR=%~dp0"

where pwsh >nul 2>nul
if errorlevel 1 (
  echo ERROR: PowerShell 7 [pwsh] was not found on PATH.
  echo Install it from https://aka.ms/powershell and re-run this file.
  exit /b 1
)

pwsh -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%Automation\PowerShell\Build_All.ps1" %*
exit /b %ERRORLEVEL%
