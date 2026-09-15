@echo off
setlocal EnableExtensions EnableDelayedExpansion
REM =====================================================================
REM  meArm one-click build entry point
REM  ATmega328P (Arduino Uno), bare-metal avr-libc firmware.
REM
REM  Usage:  start.bat              check env + compile
REM          start.bat clean        wipe .build, then check env + compile
REM          start.bat flash        compile, then flash over the bootloader
REM          start.bat flash COM3   compile, then flash on COM3
REM          start.bat -h           this help
REM
REM  Design notes
REM  ------------
REM  * ASCII only. Chinese comments break the Windows GBK console parser.
REM  * Preflight happens BEFORE any compiler is invoked: every missing
REM    component is listed in one shot, with the exact install command.
REM    "One missing tool per run" wastes a full round trip every time.
REM  * Building does not require avrdude, so avrdude is only reported as
REM    a NOTE here - a missing programmer must not block a compile.
REM  * The actual compiling lives in build.bat so there is exactly one
REM    copy of the flags and one copy of the truth.
REM  * PAUSE before EVERY exit path, success or failure alike. The
REM    scripts are meant to be double-clicked from Explorer; without it
REM    the window closes on completion and every message is lost. The
REM    pause is a contract of this script set, not a debugging leftover.
REM
REM  Parser traps handled here (all three have bitten this repo):
REM  * A bare ")" in an echo that sits inside an if/for block CLOSES the
REM    block early; the leftover text then dies with e.g.
REM    "The system cannot find the path specified." or ")" unexpected.
REM    Never echo a ")" inside a parenthesised block.
REM  * Every literal ">" printed as text must be careted: "^>".
REM  * NEVER echo a bare "!" once delayed expansion is on.
REM =====================================================================

REM  start.bat lives in the project ROOT; the actual work scripts live in
REM  scripts\. So there are two directories to track and they are NOT the
REM  same one:
REM    BATDIR -> this file's own dir (project root) - where start.bat sits
REM    SCRDIR -> the scripts\ folder  - build.bat / flash.bat live here
REM
REM  Capture BOTH up front, BEFORE any "shift": SHIFT moves %1 into %0 and
REM  %~dp0 is derived from %0, so a post-shift %~dp0 is no longer this
REM  script's directory. Measured 2026-09-15: "build.bat --no-pause" read
REM  _env.bat from the project root even though it lives in scripts\.
set "BATDIR=%~dp0"
set "SCRDIR=%~dp0scripts\"

set "NOPAUSE="
if /i "%~1"=="--no-pause" set "NOPAUSE=1" & shift

call "%SCRDIR%_env.bat"

set "HEX=%PROJ%\.build\firmware.hex"

set "ACTION=build"
set "PORT=COM4"

:parse
if "%~1"=="" goto :parsed
if /i "%~1"=="-h"     goto :usage
if /i "%~1"=="--help" goto :usage
if /i "%~1"=="/?"     goto :usage
if /i "%~1"=="clean"  (set "ACTION=clean" & shift & goto :parse)
if /i "%~1"=="flash"  (set "ACTION=flash" & shift & goto :parse)
REM a bare COM token such as COM3 / COM10 / com12 sets the flash port
echo %~1| findstr /r /i /c:"^COM[0-9][0-9]*$" >nul
if not errorlevel 1 (set "PORT=%~1" & shift & goto :parse)
echo.
echo [FAIL] Unknown argument: %~1
echo        Run "start.bat -h" for usage.
call :pause_exit
exit /b 1

:parsed

echo.
echo ===========================================================================
echo  meArm one-click build  -  ATmega328P / Arduino Uno
echo ===========================================================================
echo   project : %PROJ%

REM =====================================================================
REM  PREFLIGHT - collect ALL missing pieces before failing.
REM  Each problem is appended to its own numbered variable. Two cmd
REM  facts force this shape:
REM    * cmd has no arrays, so "MISSING_1.." + a counter is the idiom;
REM    * a ";"-joined string read back with for /f collapses into ONE
REM      line, because delims splits tokens inside a line, not lines.
REM  The counter is incremented with plain "set /a" (never delayed
REM  expansion inside a block), so it survives the whole preflight.
REM =====================================================================
set "MISSING_N=0"
set "NOTES_N=0"

if not exist "%PROJ%\core\main.cpp" set "MISSING_SRC=  [REQUIRED] project sources      core\main.cpp not found"
if not defined PIO_PKGS             set "MISSING_PKGS=  [REQUIRED] PlatformIO packages    toolchain never installed"
if not defined TOOLCHAIN_DIR        set "MISSING_TC=  [REQUIRED] toolchain-atmelavr      AVR compiler package missing"
if not defined AVR_CC               set "MISSING_CC=  [REQUIRED] avr-gcc.exe            C compiler missing"
if not defined AVR_CXX              set "MISSING_CXX=  [REQUIRED] avr-g++.exe            C++ compiler missing"
if not defined AVR_OBJCOPY          set "MISSING_OBC=  [REQUIRED] avr-objcopy.exe        Intel-hex generator missing"
if not defined AVR_SIZE             set "MISSING_SZ=  [REQUIRED] avr-size.exe           flash/ram reporter missing"

if defined MISSING_SRC  set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!MISSING_SRC!"
if defined MISSING_PKGS set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!MISSING_PKGS!"
if defined MISSING_TC   set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!MISSING_TC!"
if defined MISSING_CC   set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!MISSING_CC!"
if defined MISSING_CXX  set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!MISSING_CXX!"
if defined MISSING_OBC  set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!MISSING_OBC!"
if defined MISSING_SZ   set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!MISSING_SZ!"

if not defined AVRDUDE      set "NOTE_AVRDUDE=  [NOTE] avrdude.exe missing - compiling works, flashing does not"
if not defined AVRDUDE_CONF set "NOTE_CONF=  [NOTE] avrdude.conf missing - flashing needs it"
if not defined PYSERIAL_PY  set "NOTE_SERIAL=  [NOTE] pyserial missing - tools\host_verify.py will not run"

if defined NOTE_AVRDUDE set /a NOTES_N+=1 & set "NOTES_!NOTES_N!=!NOTE_AVRDUDE!"
if defined NOTE_CONF    set /a NOTES_N+=1 & set "NOTES_!NOTES_N!=!NOTE_CONF!"
if defined NOTE_SERIAL  set /a NOTES_N+=1 & set "NOTES_!NOTES_N!=!NOTE_SERIAL!"

set "PY_VER=not found"
if defined PY for /f "delims=" %%v in ('"%PY%" -V 2^>^&1') do set "PY_VER=%%v"
set "PIO_VER=not found"
if defined PIO for /f "delims=" %%v in ('"%PIO%" --version 2^>^&1') do set "PIO_VER=%%v"

echo.
echo  Environment
echo  -----------
echo    PlatformIO      : %PIO_VER%
echo    pio CLI         : %PIO%
echo    packages root   : %PIO_PKGS%
echo    toolchain       : %TOOLCHAIN_DIR%
echo    avrdude         : %AVRDUDE%
echo    python          : %PY_VER%
echo    pyserial via    : %PYSERIAL_PY%

if "%MISSING_N%"=="0" goto :preflight_ok

echo.
echo ===========================================================================
echo  [FAIL] Cannot build - %MISSING_N% missing component^(s^)
echo ===========================================================================
echo   Missing:
for /l %%i in (1,1,%MISSING_N%) do echo !MISSING_%%i!
echo.
echo   How to fix
echo   ----------
echo     Option A - install with PlatformIO ^(recommended, one command^):
if defined PIO (
  echo        "%PIO%" pkg install --global --platform platformio/atmelavr
) else (
  echo        1^) pip install -U platformio
  echo        2^) pio pkg install --global --platform platformio/atmelavr
)
echo.
echo     Option B - ask the WorkBuddy agent to install the dependencies.
echo        start.bat is dependency-aware; the agent can run option A for
echo        you. Just say: "install the meArm AVR build dependencies".
echo.
echo     Option C - point the scripts at another PlatformIO core dir:
echo        set PLATFORMIO_CORE_DIR=D:\path\to\pio-core
echo        the package subdir toolchain-atmelavr* is resolved automatically
echo.
echo     Option D - put a standalone avr-gcc / avr-g++ / avr-objcopy /
echo        avr-size / avrdude on PATH; the scripts fall back to PATH.
echo.
echo   Note: the avrdude package is only needed by flash.bat. A missing
echo         programmer never blocks a compile.
echo ===========================================================================
call :pause_exit
exit /b 1

:preflight_ok

if not "%NOTES_N%"=="0" (
  echo.
  echo   Notes:
  for /l %%i in (1,1,%NOTES_N%) do echo !NOTES_%%i!
)

REM ---- clean ----------------------------------------------------------
if /i "%ACTION%"=="clean" (
  echo.
  echo [1/2] Cleaning .build ...
  if exist "%PROJ%\.build" rmdir /s /q "%PROJ%\.build" 2>nul
  echo       Done.
) else (
  echo.
  echo [1/2] Incremental build - use "start.bat clean" for a full rebuild.
)

REM ---- build ----------------------------------------------------------
echo [2/2] Building ...
echo.
call "%SCRDIR%build.bat" --no-pause
set "RC=!errorlevel!"
if not "!RC!"=="0" (
  echo.
  echo [FAIL] Build failed with exit code !RC!. Nothing was uploaded.
  call :pause_exit
  exit /b !RC!
)

if /i not "%ACTION%"=="flash" goto :done

REM ---- flash (delegated, so the flashing logic exists exactly once) ---
echo.
echo [FLASH] handing over to flash.bat %PORT% ...
call "%SCRDIR%flash.bat" %PORT% --no-pause
set "RC=!errorlevel!"
if not "!RC!"=="0" (
  echo.
  echo [FAIL] Flashing failed with exit code !RC!. The firmware itself
  echo        built fine at %HEX% - check the board, port and cable.
  call :pause_exit
  exit /b !RC!
)

:done
echo ===========================================================================
echo  [OK] done
echo ===========================================================================
if /i "%ACTION%"=="flash" (
  echo   Board flashed and running. Open the serial monitor with:
  echo     monitor.bat
) else (
  echo   Firmware ready : %HEX%
  echo   Flash it with  : flash.bat %PORT%
)
echo ===========================================================================
call :pause_exit
exit /b 0

REM =====================================================================
REM  :pause_exit  (no argument)
REM  Hold the window open, unless the caller asked to suppress it. This is
REM  a bare wait: it MUST NOT be given the exit code, because "exit /b %1"
REM  inside a CALL only ends the CALL - the caller then keeps running.
REM  Every caller therefore reads:
REM
REM      call :pause_exit
REM      exit /b <code>          <- this is the one that actually returns
REM
REM  An earlier version folded both into ":pause_exit <code>" and every
REM  failure path silently carried on into the next label. Measured
REM  2026-09-15: "start.bat bogus" printed its error and then ran a full
REM  successful build, exiting 0.
REM
REM  Order matters: the pause is issued BEFORE the exit so a
REM  double-clicked window still holds the message on screen.
REM
REM  --no-pause is honoured even though this script never passes it to
REM  itself: when start.bat runs as a child of another wrapper the same
REM  contract applies. Child scripts called from here DO get --no-pause,
REM  so the user sees exactly one pause - the outermost one.
REM =====================================================================
:pause_exit
if defined NOPAUSE exit /b 0
echo.
echo Press any key to close this window . . .
pause >nul
exit /b 0

:usage
echo.
echo  meArm one-click build
echo.
echo    start.bat              check environment + compile
echo    start.bat clean        full rebuild - wipes .build first
echo    start.bat flash        compile, then flash over the bootloader
echo    start.bat flash COM3   same, but on COM3 - default COM4
echo    start.bat -h           this message
echo.
echo  The script lists every missing tool before it starts, and tells you
echo  how to install it - manually, or by asking the agent.
call :pause_exit
exit /b 0
