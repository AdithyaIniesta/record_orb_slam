@echo off
REM === EDIT THESE TWO LINES ===
set PORT=COM7
set HOURS=6
REM ============================

setlocal
cd /d "%~dp0"
call venv\Scripts\activate.bat

powercfg /change standby-timeout-ac 0 >nul 2>nul
powercfg /change monitor-timeout-ac 0 >nul 2>nul

echo [run] starting %HOURS% h capture on %PORT%. Do NOT touch the rig.
python allan_capture.py --port %PORT% --hours %HOURS%
pause
