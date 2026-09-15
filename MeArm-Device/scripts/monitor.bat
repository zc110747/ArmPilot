@echo off
setlocal EnableExtensions EnableDelayedExpansion
REM =====================================================================
REM  meArm serial monitor - PlatformIO managed, COM4 @ 115200 8N1
REM  Ctrl+C to quit.
REM
REM  Usage:  monitor.bat            open COM4
REM          monitor.bat COM3       open COM3
REM          monitor.bat -h         this help
REM
REM  PAUSE is issued before EVERY exit path. Note the monitor itself is
REM  interactive - it ends on Ctrl+C, and that is exactly when a
REM  double-clicked window would vanish, so the pause matters here too.
REM
REM  ASCII only. Chinese comments break the Windows GBK parser.
REM =====================================================================

call "%~dp0_env.bat"

if /i "%~1"=="-h"     goto :usage
if /i "%~1"=="--help" goto :usage
if /i "%~1"=="/?"     goto :usage

set "PORT=%~1"
if not defined PORT set "PORT=COM4"

if not defined PIO (
  echo.
  echo [FAIL] PlatformIO CLI ^(pio.exe^) not found - cannot open the monitor.
  echo        Expected at: %PIO_HOME%\penv\Scripts\pio.exe
  echo        Install PlatformIO Core:  pip install -U platformio
  echo.
  echo        Portable alternative without PlatformIO:
  echo          "%PIO_PY%" -m serial.tools.miniterm %PORT% 115200
  call :pause_exit
  exit /b 1
)

echo [MONITOR] %PORT% 115200 8N1  ^(Ctrl+C to quit^)
"%PIO%" device monitor -p %PORT% -b 115200
set "RC=!errorlevel!"
call :pause_exit
exit /b !RC!

REM =====================================================================
REM  :pause_exit  (no argument)
REM  Hold the window open. A monitor session that just ended (Ctrl+C, or
REM  a refused port) must not erase its own output.
REM
REM  The exit code is NOT a parameter: "exit /b %1" inside a CALL only
REM  ends the CALL, and the caller would then run on into the next label.
REM  Callers do:  call :pause_exit   /   exit /b <code>
REM =====================================================================
:pause_exit
echo.
echo Press any key to close this window . . .
pause >nul
exit /b 0

:usage
echo.
echo  meArm serial monitor
echo.
echo    monitor.bat          open COM4 @ 115200
echo    monitor.bat COM3     open COM3 @ 115200
echo    monitor.bat -h       this message
call :pause_exit
exit /b 0
