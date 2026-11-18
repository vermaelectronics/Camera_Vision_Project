@echo off
REM PASS_MVDR_STAP: build firmware + BOOT.bin from Windows Command Prompt.
REM Usage:  build.bat  [path-to-Vitis-2023.2]
REM Default Vitis path: C:\Xilinx\Vitis\2023.2
setlocal
set VITIS_DIR=%~1
if "%VITIS_DIR%"=="" set VITIS_DIR=C:\Xilinx\Vitis\2023.2
if not exist "%VITIS_DIR%\settings64.bat" (
  echo ERROR: "%VITIS_DIR%\settings64.bat" not found. Pass your Vitis 2023.2 folder:  build.bat D:\Xilinx\Vitis\2023.2
  exit /b 1
)
call "%VITIS_DIR%\settings64.bat"
cd /d "%~dp0"
call vitis -s scripts\build_fw.py
if errorlevel 1 ( echo BUILD FAILED & exit /b 1 )
if not exist output\BOOT.bin ( echo BUILD FAILED: output\BOOT.bin missing & exit /b 1 )
echo.
echo BOOT.bin ready: %~dp0output\BOOT.bin
endlocal
