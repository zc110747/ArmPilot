@echo off
setlocal EnableExtensions EnableDelayedExpansion
REM =====================================================================
REM  meArm one-click build + flash (bare-metal AVR, no Arduino framework)
REM   1) call build.bat to compile/link/produce .hex
REM   2) flash with avrdude to the target port (default COM4, override arg)
REM  Usage:
REM    build_upload.bat            -> flash to COM4
REM    build_upload.bat COM4       -> explicit port
REM    build_upload.bat COM7       -> other port
REM =====================================================================

set "PORT=COM4"
if not "%~1"=="" set "PORT=%~1"

REM ---- 1) build ----
call "%~dp0build.bat"
if errorlevel 1 (
    echo [ERROR] build failed, aborting flash
    exit /b 1
)

set "AGENT_TOOLS=D:\tools\agent-tools"
set "PKGS=%AGENT_TOOLS%\platformio-core\packages"
set "AVRDUDE_DIR="
for /d %%d in ("%PKGS%\tool-avrdude*") do set "AVRDUDE_DIR=%%d"
if not defined AVRDUDE_DIR ( echo [ERROR] tool-avrdude not found & exit /b 1 )

set "AVRDUDE=%AVRDUDE_DIR%\bin\avrdude.exe"
set "AVRDUDE_CONF=%AVRDUDE_DIR%\avrdude.conf"
set "PROJ=%~dp0.."
set "HEX=%PROJ%\.build\firmware.hex"

if not exist "%HEX%" ( echo [ERROR] %HEX% not found & exit /b 1 )

echo [UPLOAD] %HEX% -> %PORT%
"%AVRDUDE%" -C "%AVRDUDE_CONF%" -p atmega328p -c arduino -P %PORT% -b 115200 -D -U flash:w:"%HEX%":i
if errorlevel 1 (
    echo [ERROR] upload failed
    exit /b 1
)
echo [OK] build and upload complete
exit /b 0
