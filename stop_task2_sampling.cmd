@echo off
chcp 65001 >nul
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0.codex\task2_sampler.ps1" -Action Stop
if errorlevel 1 (
    echo.
    echo 任务2采样器停止失败。
    pause
    exit /b 1
)
echo.
pause
