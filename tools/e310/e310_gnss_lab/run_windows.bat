@echo off
REM Double-click: installs the packages once, then asks for the ports and starts the test.
cd /d "%~dp0"
python -m pip install -q -r requirements.txt
python e310lab.py
pause
