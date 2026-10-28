@echo off
REM Drag a Cube .BIN log onto this file to split it into three IMU CSVs.
setlocal
cd /d "%~dp0"
call venv\Scripts\activate.bat
if "%~1"=="" (
    echo Usage: drag a .BIN log onto unpack.bat
    pause & exit /b 1
)
python unpack_bin.py "%~1"
pause
