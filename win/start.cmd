@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion
cd /d "%~dp0"

REM ---------------------------------------------------------------------------
REM  gimbal_ctl (Windows) 双击启动器
REM    - 按 gimbal_ctl.ini 的配置启动
REM    - 程序异常退出时停一下，方便看清红字（正常按 Esc 退出不会停）
REM    - 想临时带参数：双击后在命令行里自己敲，例如
REM          gimbal_ctl.exe --host 10.42.0.1 --no-video --gimbal-rate 150
REM          gimbal_ctl.exe --selftest
REM ---------------------------------------------------------------------------

if not exist "gimbal_ctl.exe" (
    echo [ERROR] gimbal_ctl.exe 不在这个目录:
    echo         %CD%
    pause
    exit /b 1
)

gimbal_ctl.exe %*
set RC=!errorlevel!

if !RC! neq 0 (
    echo.
    echo ============================================================
    echo   程序异常退出 ^(exit code !RC!^)
    echo   常见原因: Jetson 上 gimbal 没跑 / IP 不对 / 网线没接
    echo   先跑一次自检看红字:  gimbal_ctl.exe --selftest
    echo ============================================================
    pause
)
endlocal
