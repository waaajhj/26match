@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0.codex\task3_sampler.ps1" -Action Stop
if errorlevel 1 (
    echo.
    echo Task3 sampler failed to stop.
    pause
    exit /b 1
)
echo.
pause
