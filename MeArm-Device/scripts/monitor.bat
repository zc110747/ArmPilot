@echo off
setlocal EnableExtensions
REM =====================================================================
REM  meArm serial monitor (PlatformIO managed): COM4 @ 9600 8N1
REM  Ctrl+C to quit
REM =====================================================================
set "AGENT_TOOLS=D:\tools\agent-tools"
set "PIO=%AGENT_TOOLS%\venv\Scripts\pio.exe"
if not exist "%PIO%" ( echo [ERROR] PlatformIO venv not found & exit /b 1 )
echo [MONITOR] COM4 9600 (Ctrl+C to quit)
"%PIO%" device monitor -p COM4 -b 9600
exit /b %errorlevel%
