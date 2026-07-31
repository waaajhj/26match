@echo off
chcp 65001 >nul
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0.codex\task2_sampler.ps1" -Action Start
if errorlevel 1 (
    echo.
    echo 任务2采样器启动失败。
    pause
    exit /b 1
)
echo.
echo 采样器已在后台运行，可以关闭本窗口。
pause
