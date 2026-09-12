@echo off
setlocal EnableExtensions
REM =====================================================================
REM  meArm flash/upload script -> Arduino Uno (ATmega328P) @ COM4, 115200 (bootloader)
REM  Usage: upload.bat [COM port]   e.g. upload.bat COM3
REM =====================================================================

set "AGENT_TOOLS=D:\tools\agent-tools"
set "PKGS=%AGENT_TOOLS%\platformio-core\packages"
set "PROJ=%~dp0.."
set "HEX=%PROJ%\.build\firmware.hex"

if not exist "%HEX%" ( echo [ERROR] %HEX% not found, run build.bat first & exit /b 1 )

set "AVRDUDE_DIR="
for /d %%d in ("%PKGS%\tool-avrdude*") do set "AVRDUDE_DIR=%%d"
if not defined AVRDUDE_DIR ( echo [ERROR] tool-avrdude not found & exit /b 1 )
REM tool-avrdude package root holds avrdude.exe / avrdude.conf directly (no bin subdir)
set "AVRDUDE=%AVRDUDE_DIR%\avrdude.exe"
set "AVRDUDE_CONF=%AVRDUDE_DIR%\avrdude.conf"

set "PORT=%~1"
if not defined PORT set "PORT=COM4"

echo [UPLOAD] %HEX% -> %PORT% (avrdude: arduino/115200)
"%AVRDUDE%" -C "%AVRDUDE_CONF%" -p atmega328p -c arduino -P %PORT% -b 115200 -D -U flash:w:"%HEX%":i
if errorlevel 1 ( echo [ERROR] upload failed & exit /b 1 )
echo [OK] upload finished
exit /b 0
