@echo off
REM One-click installer: creates a venv and installs deps.
setlocal
cd /d "%~dp0"

where python >nul 2>nul
if errorlevel 1 (
    echo [install] Python not found. Install Python 3.10+ from python.org
    echo           and tick "Add Python to PATH".
    pause & exit /b 1
)

if not exist venv (
    echo [install] creating venv...
    python -m venv venv || (echo venv creation failed & pause & exit /b 1)
)

call venv\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install -r requirements.txt || (echo pip install failed & pause & exit /b 1)

echo.
echo [install] done. Next:
echo   1. Edit run.bat and set the COM port (open Device Manager to find it).
echo   2. Mount Cube Orange on a heavy pad, let it warm 30 min untouched.
echo   3. Double-click run.bat.
pause
