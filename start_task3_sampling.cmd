@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0.codex\task3_sampler.ps1" -Action Start
if errorlevel 1 (
    echo.
    echo Task3 sampler failed to start.
    pause
    exit /b 1
)
echo.
echo Task3 sampler is waiting. Put the ball at center, then press Task3.
pause
