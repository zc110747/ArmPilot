@echo off
setlocal EnableExtensions EnableDelayedExpansion
REM =====================================================================
REM  meArm firmware flasher -> Arduino Uno (ATmega328P) over the bootloader
REM
REM  Usage:  flash.bat            flash .build\firmware.hex on COM4
REM          flash.bat COM3       flash on COM3
REM          flash.bat -h         this help
REM
REM  Preflight: the hex file, avrdude, avrdude.conf and the COM port are
REM  all checked BEFORE avrdude runs, and every problem found is listed
REM  in one pass. A missing board is reported as a clear message, not as
REM  a raw avrdude error dump.
REM
REM  PAUSE is issued before EVERY exit path. This script is meant to be
REM  double-clicked, and a failed upload is exactly the case where you
REM  must be able to read the diagnosis. Pass --no-pause to suppress it
REM  when another script drives this one and owns the final pause.
REM
REM  ASCII only. Chinese comments break the Windows GBK parser.
REM =====================================================================

REM  Capture this script's own directory BEFORE any "shift": SHIFT moves
REM  %1 into %0 and %~dp0 is derived from %0, so a post-shift %~dp0 is no
REM  longer this script's directory. See build.bat for the full note.
set "BATDIR=%~dp0"
set "NOPAUSE="
if /i "%~1"=="--no-pause" set "NOPAUSE=1" & shift

call "%BATDIR%_env.bat"

set "HEX=%PROJ%\.build\firmware.hex"

if /i "%~1"=="-h"     goto :usage
if /i "%~1"=="--help" goto :usage
if /i "%~1"=="/?"     goto :usage

set "PORT=%~1"
if not defined PORT set "PORT=COM4"

REM ---- port sanity: catch "flash.bat 4" / "flash.bat com4 " typos ------
echo %PORT%| findstr /r /i /c:"^COM[0-9][0-9]*$" >nul
if errorlevel 1 (
  echo.
  echo [FAIL] Invalid COM port: "%PORT%"
  echo        Expected a name like COM4 or COM12.
  echo        Usage: flash.bat [COM port]      e.g. flash.bat COM3
  call :pause_exit
  exit /b 1
)

echo.
echo ===========================================================================
echo  meArm flash  -  %HEX%
echo ===========================================================================
echo   port      : %PORT%
echo   avrdude   : %AVRDUDE%
echo   config    : %AVRDUDE_CONF%

REM =====================================================================
REM  PREFLIGHT - collect every missing piece, then decide.
REM  Each problem is its own numbered slot: cmd has no arrays, and a
REM  ";"-joined string read back with for /f collapses into ONE line.
REM =====================================================================
set "MISSING_N=0"
set "PORT_OK="
echo %PORT_LIST%| findstr /i /c:",%PORT%," >nul
if not errorlevel 1 set "PORT_OK=1"

if not exist "%HEX%"        set "P1=  [REQUIRED] firmware hex not found: %HEX%"
if not exist "%HEX%"        set "P2=      run start.bat or build.bat first"
if not defined AVRDUDE      set "P3=  [REQUIRED] avrdude.exe not found"
if not defined AVRDUDE      set "P4=      pio pkg install --global --tool platformio/tool-avrdude"
if not defined AVRDUDE_CONF set "P5=  [REQUIRED] avrdude.conf not found"
if not defined AVRDUDE_CONF set "P6=      it ships with the tool-avrdude package - reinstall it"
if not defined PORT_OK      set "P7=  [REQUIRED] COM port %PORT% is not present"
if not defined PORT_OK      set "P8=      plug the board in, then re-run; or pass another port: flash.bat COM7"
if not defined PORT_OK      set "P9=      ports currently present: %PORT_LIST%"

if defined P1 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!P1!"
if defined P2 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!P2!"
if defined P3 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!P3!"
if defined P4 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!P4!"
if defined P5 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!P5!"
if defined P6 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!P6!"
if defined P7 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!P7!"
if defined P8 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!P8!"
if defined P9 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!P9!"

if "%MISSING_N%"=="0" goto :preflight_ok

echo.
echo ===========================================================================
echo  [FAIL] Cannot flash - %MISSING_N% preflight problem^(s^)
echo ===========================================================================
for /l %%i in (1,1,%MISSING_N%) do echo !MISSING_%%i!
echo.
echo   Nothing was uploaded - the board was NOT touched.
echo ===========================================================================
call :pause_exit
exit /b 1

:preflight_ok

REM ---- Courtesy check: does the port look like a USB-serial adapter? ----
REM  WARNING 1: never put a non-ASCII literal in a findstr /c: pattern here.
REM  This file is read by the GBK console parser; a UTF-8 Chinese string
REM  corrupts the rest of the LINE, which splits it into fragments cmd
REM  then tries to run as commands (seen 2026-09-15: the whole script died
REM  with "'local' is not recognized..."). Match ASCII tokens only.
REM
REM  WARNING 2: inside for /f backquotes the cmd metacharacters must be
REM  escaped, including the pipe in a PowerShell pipeline. A bare "|" there
REM  ends the command substitution and cmd then reports
REM  "The syntax of the command is incorrect." Use "^|" and "^>".
REM
REM  WARNING 3: this is only a COURTESY warning. A genuine Uno reports
REM  "USB Serial Device" with no vendor token at all, so we never block on
REM  this string - the real avrdude result is what decides.
set "PORT_DESC="
set "PS_Q=Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue ^| Where-Object { $_.Name -match 'COM%~1$' } ^| Select-Object -First 1 -ExpandProperty Name"
REM  %~1 is used instead of %PORT:~3% so the match is a plain suffix and
REM  cannot be confused by ports whose number is a prefix of another.
for /f "usebackq delims=" %%D in (`powershell -NoProfile -Command "%PS_Q%" 2^>nul`) do if not defined PORT_DESC set "PORT_DESC=%%D"
if not defined PORT_DESC goto :after_port_desc
echo   device    : !PORT_DESC!
echo !PORT_DESC!| findstr /i /c:"Arduino" /c:"CH340" /c:"CH341" /c:"CP210" /c:"FTDI" /c:"USB Serial" >nul
if errorlevel 1 (
  echo.
  echo   [WARN] "%PORT%" does not look like an Arduino or USB-serial adapter.
  echo          If the upload fails, check that you picked the right port.
)
:after_port_desc

echo.
echo [FLASH] avrdude  -p atmega328p -c arduino -P %PORT% -b 115200 -U flash:w
echo.
"%AVRDUDE%" -C "%AVRDUDE_CONF%" -p atmega328p -c arduino -P %PORT% -b 115200 -D -U flash:w:"%HEX%":i
set "RC=!errorlevel!"

if not "!RC!"=="0" (
  echo.
  echo ===========================================================================
  echo  [FAIL] avrdude returned !RC! - upload did NOT succeed
  echo ===========================================================================
  echo   The firmware file is fine; this is a transport problem. Check:
  echo     1. the board is plugged in and you picked its port
  echo     2. no serial monitor / host_verify.py holds the port
  echo        a busy port is the #1 cause: "access denied"
  echo     3. bootloader alive - press RESET on the Uno, then retry
  echo     4. correct driver - CH340 / ATmega16U2 / FTDI
  echo ===========================================================================
  call :pause_exit
  exit /b !RC!
)

echo.
echo ===========================================================================
echo  [OK] flashed %PORT% - the sketch is running
echo ===========================================================================
echo   Serial monitor : monitor.bat
echo   Host self-test : python tools\host_verify.py %PORT% 115200
echo.
echo   Note: platformio.ini pins COM4. If you flashed %PORT% and want the
echo         monitor / self-test to default to it, update monitor_port and
echo         upload_port there.
call :pause_exit
exit /b 0

REM =====================================================================
REM  :pause_exit  (no argument)
REM  Hold the window open before returning 0, unless --no-pause was given
REM  by a parent script. Without this the Explorer double-click closes the
REM  window and every message above is lost.
REM
REM  The code is deliberately NOT a parameter: "exit /b %1" inside a CALL
REM  only ends the CALL, so a caller written as "call :pause_exit 1" would
REM  pause and then keep on running into the next label. Callers use
REM
REM      call :pause_exit
REM      exit /b <code>
REM
REM  with the real exit left to the caller. Measured 2026-09-15: the
REM  folded form made a failed preflight report exit code 0.
REM =====================================================================
:pause_exit
if defined NOPAUSE exit /b 0
echo.
echo Press any key to close this window . . .
pause >nul
exit /b 0

:usage
echo.
echo  meArm flash script
echo.
echo    flash.bat            flash .build\firmware.hex on COM4
echo    flash.bat COM3       flash on COM3
echo    flash.bat -h         this message
echo.
echo  Build first with start.bat if .build\firmware.hex is missing.
call :pause_exit
exit /b 0
