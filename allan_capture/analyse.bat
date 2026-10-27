@echo off
REM Drag a captured imu*.csv onto this file, or pass the path as an argument.
setlocal
cd /d "%~dp0"
call venv\Scripts\activate.bat
if "%~1"=="" (
    echo Usage: drag an imu0.csv onto analyse.bat
    pause & exit /b 1
)
python allan_fit.py "%~1"
pause
