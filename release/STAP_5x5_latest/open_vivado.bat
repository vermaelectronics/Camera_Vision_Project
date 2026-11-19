@echo off
REM Open the PASS_MVDR_STAP Vivado project. Usage: open_vivado.bat [path-to-Vivado-2023.2]
set VIVADO_DIR=%~1
if "%VIVADO_DIR%"=="" set VIVADO_DIR=C:\Xilinx\Vivado\2023.2
call "%VIVADO_DIR%\settings64.bat"
cd /d "%~dp0hdl\Passthrough_MVDR"
start vivado passthrough_mvdr.xpr
