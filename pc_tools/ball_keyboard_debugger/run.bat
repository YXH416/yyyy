@echo off
cd /d "%~dp0"
where py >nul 2>nul
if not errorlevel 1 (
    py -3 app.py
    if not errorlevel 1 exit /b 0
)
where python >nul 2>nul
if not errorlevel 1 (
    python app.py
    if not errorlevel 1 exit /b 0
)
echo Python 3 was not found. Install Python 3 and pyserial first.
pause
