@echo off
rem =====================================================================
rem  meArm shared environment resolver
rem  Sourced by start.bat / build.bat / flash.bat / monitor.bat / clean.bat
rem  ASCII only - Chinese comments break the GBK console parser.
rem
rem  Sets: AVR_CC AVR_CXX AVR_OBJCOPY AVR_SIZE AVRDUDE AVRDUDE_CONF
rem        PIO  PIO_PY  PYSERIAL_PY  PY  MINGW_BIN  PIO_PKGS  PROJ
rem        TOOLCHAIN_DIR  AVRDUDE_DIR
rem
rem  MUST be called, never executed directly:
rem      call "%~dp0_env.bat"
rem  Every "set" here is intentionally GLOBAL (no setlocal) so the
rem  caller inherits the resolved paths.
rem =====================================================================

REM  All .bat files live in <PROJ>\scripts\, so THIS file's directory is
REM  the "scripts" folder - the project root is its PARENT.
REM
REM  Do NOT use "%~dp0.." literally: when the parent HAS a trailing
REM  component cmd gives back "...\scripts\.." with a surviving backslash
REM  and mixed separators, which then leak into every downstream path.
REM  pushd/popd asks the OS for the canonical absolute path instead, so
REM  the ".." is collapsed for real.
REM
REM  Layout guard: climb to the parent only when THIS file's own folder is
REM  named "scripts" AND that parent really is the project (marker file
REM  present). Measured 2026-09-15: two traps here.
REM    1. "%%~nxd" must be read from "%~dp0.", NOT from "%~dp0..". The
REM       parent-of-scripts is "MeArm-Device"; testing THAT for "scripts"
REM       never matches and PROJ silently stays at scripts\.
REM    2. Without the core\main.cpp marker test, dropping the .bat files
REM       back into the project root would resolve PROJ one level ABOVE
REM       the project ("\MeArmPilot") and the build would wrongly report
REM       core\main.cpp as missing.
REM  Why "%~dp0." : "%~dp0" already ends in a backslash, and "%%~nxd" of a
REM  name ending in "\" yields the name itself, so the trailing dot form
REM  is what makes the final component come back as "scripts".
set "PROJ=%~dp0"
for %%s in ("%~dp0.") do if /i "%%~nxs"=="scripts" (
  for %%d in ("%~dp0..") do if exist "%%~fd\core\main.cpp" set "PROJ=%%~fd"
)
pushd "%PROJ%" >nul 2>nul
set "PROJ=%CD%"
popd >nul 2>nul

REM ---- 1. PlatformIO package root -------------------------------------
REM  Preference order:
REM    a) %PLATFORMIO_CORE_DIR%\packages   (explicit override)
REM    b) %USERPROFILE%\.platformio\packages (standard install - what this PC has)
REM    c) D:\tools\agent-tools\platformio-core\packages (legacy layout)
REM  The version subdir (toolchain-atmelavr@x.y.z) is resolved by glob.
REM  Done: see resolve_toolchain below, which handles both the flat and
REM  the nested layout.
set "PIO_HOME=%USERPROFILE%\.platformio"
if defined PLATFORMIO_CORE_DIR set "PIO_HOME=%PLATFORMIO_CORE_DIR%"

set "PIO_PKGS="
if exist "%PIO_HOME%\packages" set "PIO_PKGS=%PIO_HOME%\packages"
if not defined PIO_PKGS if exist "D:\tools\agent-tools\platformio-core\packages" (
  set "PIO_PKGS=D:\tools\agent-tools\platformio-core\packages"
)

REM ---- 2. Toolchain + avrdude -----------------------------------------
REM  Two layouts exist in the wild and BOTH must be handled:
REM    * flat     : packages\toolchain-atmelavr\bin\avr-gcc.exe
REM                 (what this PC has after "pio pkg install --global")
REM    * nested   : packages\toolchain-atmelavr@1.7.0\<ver>\bin\...
REM                 (the older per-version layout)
REM  So probe direct children FIRST, then one level deeper. Getting this
REM  order wrong is silent: the deep glob can match a sibling directory
REM  such as i686-w64-mingw32\, which has no bin\avr-gcc.exe at all.
REM  NB: a matching directory comes back WITH surrounding quotes, hence
REM  the second "for" that strips them via %%~d.
set "TOOLCHAIN_DIR="
set "AVRDUDE_DIR="
if defined PIO_PKGS call :resolve_toolchain

REM  Jump straight past the helper definitions on the normal path.
goto :after_helpers

REM ---- helper: resolve_toolchain --------------------------------------
REM  Probe for the flat layout first (packages\<pkg>\bin\...), then the
REM  nested one (packages\<pkg>@<ver>\<version>\bin\...). Both are real:
REM  "pio pkg install --global" produces flat on this machine, older
REM  PlatformIO releases produced nested.
REM  This CANNOT be written as a call inside a parenthesised block -
REM  cmd resolves call labels against the file image, and a label below
REM  the call site is not reliably available from inside a block.
:resolve_toolchain
for /d %%d in ("%PIO_PKGS%\toolchain-atmelavr*") do (
  if not defined TOOLCHAIN_DIR if exist "%%~d\bin\avr-gcc.exe" set "TOOLCHAIN_DIR=%%~d"
)
if not defined TOOLCHAIN_DIR for /d %%p in ("%PIO_PKGS%\toolchain-atmelavr*") do (
  for /d %%d in ("%%~p\*") do (
    if not defined TOOLCHAIN_DIR if exist "%%~d\bin\avr-gcc.exe" set "TOOLCHAIN_DIR=%%~d"
  )
)

for /d %%d in ("%PIO_PKGS%\tool-avrdude*") do (
  if not defined AVRDUDE_DIR if exist "%%~d\avrdude.exe" set "AVRDUDE_DIR=%%~d"
)
if not defined AVRDUDE_DIR for /d %%p in ("%PIO_PKGS%\tool-avrdude*") do (
  for /d %%d in ("%%~p\*") do (
    if not defined AVRDUDE_DIR if exist "%%~d\avrdude.exe" set "AVRDUDE_DIR=%%~d"
  )
)
goto :eof

:after_helpers

set "AVR_CC="
set "AVR_CXX="
set "AVR_OBJCOPY="
set "AVR_SIZE="
if defined TOOLCHAIN_DIR (
  if exist "%TOOLCHAIN_DIR%\bin\avr-gcc.exe"     set "AVR_CC=%TOOLCHAIN_DIR%\bin\avr-gcc.exe"
  if exist "%TOOLCHAIN_DIR%\bin\avr-g++.exe"     set "AVR_CXX=%TOOLCHAIN_DIR%\bin\avr-g++.exe"
  if exist "%TOOLCHAIN_DIR%\bin\avr-objcopy.exe" set "AVR_OBJCOPY=%TOOLCHAIN_DIR%\bin\avr-objcopy.exe"
  if exist "%TOOLCHAIN_DIR%\bin\avr-size.exe"    set "AVR_SIZE=%TOOLCHAIN_DIR%\bin\avr-size.exe"
)

set "AVRDUDE="
set "AVRDUDE_CONF="
if defined AVRDUDE_DIR (
  REM tool-avrdude keeps avrdude.exe at the package root; some builds
  REM nest it under bin\. Probe both, root first (matches this install).
  if exist "%AVRDUDE_DIR%\avrdude.exe"     set "AVRDUDE=%AVRDUDE_DIR%\avrdude.exe"
  if not defined AVRDUDE if exist "%AVRDUDE_DIR%\bin\avrdude.exe" set "AVRDUDE=%AVRDUDE_DIR%\bin\avrdude.exe"
  if exist "%AVRDUDE_DIR%\avrdude.conf"     set "AVRDUDE_CONF=%AVRDUDE_DIR%\avrdude.conf"
  if not defined AVRDUDE_CONF if exist "%AVRDUDE_DIR%\bin\avrdude.conf" set "AVRDUDE_CONF=%AVRDUDE_DIR%\bin\avrdude.conf"
)

REM ---- 3. Fallback: whatever happens to be on PATH ---------------------
REM  Only consulted when the PlatformIO packages are absent. The "for"
REM  variable is written UNQUOTED (%%p, not "%%p") on purpose: a quoted
REM  for-variable leaks the surrounding quotes into the value, which
REM  makes every later "if exist" test fail. Measured 2026-09-15.
REM
REM  DANGER - a PATH hit is NOT automatically a usable toolchain.
REM  This PC has Microchip XC8 on PATH, so "avr-objcopy" resolves to
REM    C:\Program Files\Microchip\xc8\v3.10\bin\avr-objcopy
REM  That binary belongs to a different compiler family. Feeding it into
REM  an avr-gcc link produces confusing hex/link failures, so a PATH
REM  toolchain from an unrelated vendor is REJECTED outright rather than
REM  used silently. Better a clear "install it" than a mystery error.
set "EXTERNAL_TOOLCHAIN="
set "REJECTED_CC="

if not defined AVR_CC     for %%p in (avr-gcc.exe)     do if not "%%~$PATH:p"=="" set "AVR_CC=%%~$PATH:p"
if not defined AVR_CXX    for %%p in (avr-g++.exe)     do if not "%%~$PATH:p"=="" set "AVR_CXX=%%~$PATH:p"
if not defined AVR_OBJCOPY for %%p in (avr-objcopy.exe) do if not "%%~$PATH:p"=="" set "AVR_OBJCOPY=%%~$PATH:p"
if not defined AVR_SIZE   for %%p in (avr-size.exe)    do if not "%%~$PATH:p"=="" set "AVR_SIZE=%%~$PATH:p"
if not defined AVRDUDE    for %%p in (avrdude.exe)     do if not "%%~$PATH:p"=="" set "AVRDUDE=%%~$PATH:p"

if not defined AVR_CC goto :env_done_fallback
if not defined AVR_CXX goto :env_done_fallback
if not defined AVR_OBJCOPY goto :env_done_fallback
if not defined AVR_SIZE goto :env_done_fallback

REM  All four came from PATH, so note that this is NOT the managed set.
set "EXTERNAL_TOOLCHAIN=PATH"

REM  Reject the whole set when the compiler or helpers come from a vendor
REM  toolchain that has nothing to do with AVR 8-bit GCC.
echo %AVR_CC% %AVR_OBJCOPY%| findstr /i /c:"\\Microchip\\" /c:"\\xc8\\" /c:"\\xc16\\" /c:"\\Keil" >nul
if not errorlevel 1 (
  set "REJECTED_CC=%AVR_CC%"
  set "AVR_CC="
  set "AVR_CXX="
  set "AVR_OBJCOPY="
  set "AVR_SIZE="
  set "EXTERNAL_TOOLCHAIN=REJECTED"
)

:env_done_fallback
if not defined AVRDUDE goto :env_done
echo %AVRDUDE%| findstr /i /c:"\\Microchip\\" /c:"\\xc8\\" /c:"\\xc16\\" >nul
if not errorlevel 1 set "AVRDUDE="

:env_done

REM ---- 4. PlatformIO CLI (monitor / package installer) -----------------
set "PIO=%PIO_HOME%\penv\Scripts\pio.exe"
if not exist "%PIO%" set "PIO="
if not defined PIO for %%p in (pio.exe) do if not "%%~$PATH:p"=="" set "PIO=%%~$PATH:p"

set "PIO_PY=%PIO_HOME%\penv\Scripts\python.exe"
if not exist "%PIO_PY%" set "PIO_PY="

REM ---- 5. Host python + pyserial (tools\host_verify.py) ---------------
REM  PlatformIO's own venv already bundles pyserial, so prefer it --
REM  that also matches the interpreter the firmware was verified with.
set "PY="
for %%p in (python.exe) do if not "%%~$PATH:p"=="" set "PY=%%~$PATH:p"
set "PYSERIAL_PY="
if defined PIO_PY (
  "%PIO_PY%" -c "import serial" >nul 2>nul
  if not errorlevel 1 set "PYSERIAL_PY=%PIO_PY%"
)
if not defined PYSERIAL_PY if defined PY (
  "%PY%" -c "import serial" >nul 2>nul
  if not errorlevel 1 set "PYSERIAL_PY=%PY%"
)

REM ---- 6. Optional: MinGW gcc (host-side helper builds) ---------------
set "MINGW_BIN="
if exist "D:\Software\MSYS2\mingw64\bin\gcc.exe" set "MINGW_BIN=D:\Software\MSYS2\mingw64\bin"

REM ---- 7. COM port enumeration ----------------------------------------
REM  PORT_LIST holds the currently present serial ports as ",COM3,COM4,".
REM  Callers test membership with:
REM      echo %PORT_LIST%| findstr /i /c:",COM4," >nul
REM
REM  Why not `if exist "\\.\COM4"`: on this machine that returns "not
REM  exist" for EVERY port, including listed and working ones (verified
REM  2026-09-15 across COM3/4/9/10/19/20/21). Relying on it would refuse
REM  to flash a perfectly good board.
set "PORT_LIST="
for /f "usebackq tokens=*" %%P in (`powershell -NoProfile -Command "[System.IO.Ports.SerialPort]::GetPortNames() -join ','" 2^>nul`) do set "PORT_LIST=,%%P,"

exit /b 0
