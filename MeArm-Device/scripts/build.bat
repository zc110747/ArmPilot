@echo off
setlocal EnableExtensions EnableDelayedExpansion
REM =====================================================================
REM  meArm bare-metal AVR compiler
REM  ATmega328P (Arduino Uno), pure avr-libc, no Arduino framework.
REM
REM  Usage:  build.bat          compile + link + hex + size report
REM          build.bat clean    remove .build first, then compile
REM          build.bat -h       this help
REM
REM  Toolchain is provided by PlatformIO packages. The install location
REM  is resolved by _env.bat (see it for the search order).
REM  Nothing is hardcoded to one drive any more.
REM
REM  PAUSE is issued before EVERY exit path, so a double-click from
REM  Explorer leaves the errors readable. Pass --no-pause when a parent
REM  script (start.bat) drives this one and owns the final pause.
REM
REM  ASCII only - Chinese comments break the Windows GBK parser.
REM =====================================================================

set "NOPAUSE="
REM  Capture this script's own directory BEFORE any "shift". SHIFT moves
REM  %1 into %0, and %~dp0 is derived from %0 - so after a shift, %~dp0
REM  silently degrades (with no arguments left it resolves against the
REM  current directory instead). Measured 2026-09-15: "build.bat
REM  --no-pause" reported _env.bat from the PROJECT ROOT even though the
REM  script lives in scripts\, breaking every path in this file.
set "BATDIR=%~dp0"
if /i "%~1"=="--no-pause" set "NOPAUSE=1" & shift

call "%BATDIR%_env.bat"

set "OUT=%PROJ%\.build"
set "MCU=atmega328p"
set "FCPU=16000000"

if /i "%~1"=="-h"     goto :usage
if /i "%~1"=="--help" goto :usage
if /i "%~1"=="/?"     goto :usage

REM =====================================================================
REM  PREFLIGHT: report every missing component, not just the first one.
REM  Each problem gets a numbered slot - a joined string would collapse
REM  into a single line when read back with for /f.
REM =====================================================================
set "MISSING_N=0"

if not exist "%PROJ%\core\main.cpp" set "M1=   - core\main.cpp (project sources)"
if not defined PIO_PKGS             set "M2=   - PlatformIO packages directory (toolchain NOT installed)"
if not defined AVR_CC               set "M3=   - avr-gcc.exe     (C compiler)"
if not defined AVR_CXX              set "M4=   - avr-g++.exe     (C++ compiler)"
if not defined AVR_OBJCOPY          set "M5=   - avr-objcopy.exe (hex generation)"
if not defined AVR_SIZE             set "M6=   - avr-size.exe    (flash/ram report)"

if defined M1 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!M1!"
if defined M2 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!M2!"
if defined M3 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!M3!"
if defined M4 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!M4!"
if defined M5 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!M5!"
if defined M6 set /a MISSING_N+=1 & set "MISSING_!MISSING_N!=!M6!"

if "%MISSING_N%"=="0" goto :preflight_ok

echo.
echo ===========================================================================
echo  [FAIL] Build cannot start - %MISSING_N% missing component^(s^)
echo ===========================================================================
echo  Missing:
for /l %%i in (1,1,%MISSING_N%) do echo !MISSING_%%i! & set "MISSING_%%i="
echo.
echo  Detected environment
echo  --------------------
echo    USERPROFILE           : %USERPROFILE%
echo    PlatformIO home       : %PIO_HOME%
echo    PlatformIO packages   : %PIO_PKGS%
echo    toolchain-atmelavr    : %TOOLCHAIN_DIR%
echo    pio CLI               : %PIO%
echo.
echo  How to fix
echo  ----------
echo    Option A - let PlatformIO install the toolchain (recommended):
if defined PIO (
  echo        "%PIO%" pkg install --global --platform platformio/atmelavr
) else (
  echo        1^) install PlatformIO Core:  pip install -U platformio
  echo        2^) then run:  pio pkg install --global --platform platformio/atmelavr
)
echo.
echo    Option B - let the WorkBuddy agent install it for you.
echo        This script is already dependency-aware: just ask the agent to
echo        "install the meArm build dependencies" and re-run start.bat.
echo.
echo    Option C - use an external toolchain already on PATH:
echo        put avr-gcc.exe / avr-g++.exe / avr-objcopy.exe / avr-size.exe
echo        into PATH, or set PLATFORMIO_CORE_DIR to your pio core dir.
echo.
echo    Note: build.bat checks avrdude only in flash.bat - compiling does
echo          not need a programmer.
echo ===========================================================================
call :pause_exit
exit /b 1

:preflight_ok

if /i "%~1"=="clean" (
  if exist "%OUT%" rmdir /s /q "%OUT%" 2>nul
  echo [CLEAN] removed %OUT%
)

echo ===========================================================================
echo  meArm build - ATmega328P @ %FCPU% Hz
echo ===========================================================================
echo   toolchain : %TOOLCHAIN_DIR%
echo   project   : %PROJ%
echo.

if not exist "%OUT%" mkdir "%OUT%" >nul 2>nul

set "CFLAGS=-mmcu=%MCU% -DF_CPU=%FCPU%UL -Os -Wall -Wextra -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -I. -Ibsp -Icore"
set "CXXFLAGS=%CFLAGS% -fno-exceptions -fno-rtti"

REM ---- compile every .c, then core\main.cpp ---------------------------
set "OBJS="
for %%f in (bsp\*.c core\*.c) do (
  set "o=%OUT%\%%~nf.o"
  echo   CC   %%f
  "%AVR_CC%" %CFLAGS% -c "%%f" -o "!o!"
  if errorlevel 1 goto :compile_failed
  set "OBJS=!OBJS! !o!"
)
echo   CXX  core\main.cpp
"%AVR_CXX%" %CXXFLAGS% -c "core\main.cpp" -o "%OUT%\main.o"
if errorlevel 1 goto :compile_failed
set "OBJS=!OBJS! %OUT%\main.o"

REM ---- link -----------------------------------------------------------
echo   LD   firmware.elf
"%AVR_CC%" -mmcu=%MCU% !OBJS! -o "%OUT%\firmware.elf" -Wl,-Map="%OUT%\firmware.map" -lm
if errorlevel 1 goto :link_failed

"%AVR_OBJCOPY%" -O ihex -R .eeprom "%OUT%\firmware.elf" "%OUT%\firmware.hex"
if errorlevel 1 goto :hex_failed

REM ---- size report ----------------------------------------------------
REM  Parse a TEMP FILE rather than a nested command pipe. Doing this with
REM    for /f ... in ('"tool" -A "elf" ^| findstr ...')
REM  fails on this toolchain ("The filename, directory name or volume
REM  label syntax is incorrect") because cmd mangles the quoted exe path
REM  inside the command substitution. Dumping to a file removes cmd from
REM  the equation entirely.
set /a TEXT=0
set /a DATA=0
set /a BSS=0
"%AVR_SIZE%" -A "%OUT%\firmware.elf" > "%OUT%\size.txt" 2>nul
if not exist "%OUT%\size.txt" goto :size_done
for /f "usebackq tokens=1,2" %%a in ("%OUT%\size.txt") do (
  if "%%a"==".text" set /a TEXT=%%b
  if "%%a"==".data" set /a DATA=%%b
  if "%%a"==".bss"  set /a BSS=%%b
)
:size_done
set /a FLASH=TEXT+DATA
set /a RAM=DATA+BSS

echo.
echo ===========================================================================
echo  [OK] build finished
echo ===========================================================================
echo   hex   : %OUT%\firmware.hex
echo   FLASH : %FLASH% B  (text %TEXT% + data %DATA%) / 32256 B usable
echo   RAM   : %RAM% B  (data %DATA% + bss %BSS%) / 2048 B
echo.
echo   Next:  flash.bat  [COM port]     (default COM4)
echo ===========================================================================
call :pause_exit
exit /b 0

REM  Each failure label below is a normal GOTO destination, so it must end
REM  with its own "exit /b". Without it cmd simply FALLS THROUGH into the
REM  next label, and one failed compile prints all three failures in a row.
REM  Measured 2026-09-15: the success path printed "[OK] build finished"
REM  and then immediately "[FAIL] compilation failed" / "link failed" /
REM  "hex generation failed", because "call :pause_exit <code>" RETURNS
REM  instead of terminating the script.
:compile_failed
echo.
echo [FAIL] compilation failed. Fix the errors above and re-run.
call :pause_exit
exit /b 1

:link_failed
echo.
echo [FAIL] link failed.
call :pause_exit
exit /b 1

:hex_failed
echo.
echo [FAIL] hex generation failed.
call :pause_exit
exit /b 1

REM =====================================================================
REM  :pause_exit  (no argument)
REM  Hold the window open, unless --no-pause was given by a parent script
REM  (start.bat), which then owns the single visible pause.
REM
REM  The exit code is NOT a parameter on purpose: "exit /b %1" inside a
REM  CALL only ends the CALL, so the caller must issue its own
REM      call :pause_exit
REM      exit /b <code>
REM =====================================================================
:pause_exit
if defined NOPAUSE exit /b 0
echo.
echo Press any key to close this window . . .
pause >nul
exit /b 0

:usage
echo.
echo  meArm build script
echo.
echo    build.bat          compile + link + hex + size report
echo    build.bat clean    wipe .build first, then compile
echo    build.bat -h       this help
echo.
echo  Toolchain is resolved automatically from the PlatformIO packages
echo  directory. Override it with the PLATFORMIO_CORE_DIR env var.
call :pause_exit
exit /b 0
