@echo off
REM ─────────────────────────────────────────────────────────────────────
REM  Rig Capture - build a self-contained Windows .exe  (same recipe as jvp-ui / CvTrackerGroundStation)
REM
REM  Run ONCE on a dev PC with internet access.  Produces:  dist\RigCapture.exe
REM
REM  To use it on another PC / laptop copy ONLY  RigCapture.exe  and double-click it.
REM  That PC needs: GStreamer (gst-launch-1.0 on PATH, same as for jvp-ui) and the Windows
REM  "OpenSSH Client" (built into Windows 10/11).  No Python, no admin rights.
REM  The Jetson side (stereo_capture.py) is bundled inside the exe and uploaded to the Jetson by the app.
REM ─────────────────────────────────────────────────────────────────────
setlocal
cd /d "%~dp0"

REM Find a real Python (plain "python" is often the Microsoft Store stub).
set PYEXE=
where py >nul 2>nul && set PYEXE=py -3
if not defined PYEXE if exist "%LocalAppData%\Programs\Python\Python311\python.exe" set PYEXE="%LocalAppData%\Programs\Python\Python311\python.exe"
if not defined PYEXE if exist "%LocalAppData%\Programs\Python\Python312\python.exe" set PYEXE="%LocalAppData%\Programs\Python\Python312\python.exe"
if not defined PYEXE if exist "%LocalAppData%\Programs\Python\Python313\python.exe" set PYEXE="%LocalAppData%\Programs\Python\Python313\python.exe"
if not defined PYEXE set PYEXE=python

if not exist ".venv\Scripts\python.exe" (
    echo Creating virtualenv with %PYEXE% ...
    %PYEXE% -m venv .venv
    if errorlevel 1 (
        echo ERROR: no usable Python found. Install from python.org and check "Add to PATH".
        exit /b 1
    )
)

echo Installing / refreshing dependencies...
.venv\Scripts\python.exe -m pip install --upgrade pip --quiet
.venv\Scripts\python.exe -m pip install --quiet -r requirements.txt
if errorlevel 1 (
    echo ERROR: pip install failed. Do you have internet for this build?
    exit /b 1
)

REM Take the newest Jetson-side program from the repo if it is next to this folder.
set REPO_SCRIPT=%~dp0..\jetson-tracking-perception\scripts\stereo_capture\stereo_capture.py
if exist "%REPO_SCRIPT%" (
    echo Updating jetson\stereo_capture.py from the repo...
    copy /y "%REPO_SCRIPT%" "jetson\stereo_capture.py" >nul
)
if not exist "jetson\stereo_capture.py" (
    echo ERROR: jetson\stereo_capture.py is missing.
    exit /b 1
)

REM Optional: drop an ffmpeg.exe next to this .bat to bundle it (fallback decoder when GStreamer is absent).
set FFMPEG_ARG=
if exist "ffmpeg.exe" set FFMPEG_ARG=--add-binary "ffmpeg.exe;."

set ICON_ARG=
if exist "rig_capture.ico" set ICON_ARG=--icon "rig_capture.ico"

if exist "dist\RigCapture.exe" del /q "dist\RigCapture.exe"
echo.
echo Building single-file executable... this takes a minute or two.
.venv\Scripts\python.exe -m PyInstaller ^
    --onefile ^
    --noconsole ^
    --clean ^
    --name RigCapture ^
    --distpath dist ^
    %ICON_ARG% ^
    --add-data "jetson\stereo_capture.py;." ^
    %FFMPEG_ARG% ^
    rig_capture.py
if errorlevel 1 (
    echo BUILD FAILED
    exit /b 1
)

echo.
echo ─────────────────────────────────────────────────────────
echo  Built: dist\RigCapture.exe
echo ─────────────────────────────────────────────────────────
dir "dist\RigCapture.exe"
echo.
echo Copy ONLY dist\RigCapture.exe to the other PC and double-click it.
