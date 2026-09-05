@echo off
setlocal EnableExtensions
REM =====================================================================
REM  meArm 串口监视脚本 (PlatformIO 管理): COM4 @ 9600 8N1
REM  Ctrl+C 退出
REM =====================================================================
set "AGENT_TOOLS=D:\tools\agent-tools"
set "PIO=%AGENT_TOOLS%\venv\Scripts\pio.exe"
if not exist "%PIO%" ( echo [ERROR] 未找到 PlatformIO venv & exit /b 1 )
echo [MONITOR] COM4 9600 (Ctrl+C to quit)
"%PIO%" device monitor -p COM4 -b 9600
exit /b %errorlevel%
