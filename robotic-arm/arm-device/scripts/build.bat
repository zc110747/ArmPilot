@echo off
setlocal EnableExtensions EnableDelayedExpansion
REM =====================================================================
REM  meArm 裸机 AVR 构建脚本 (无 Arduino 框架, 纯 avr-libc + 寄存器)
REM  工具链由 PlatformIO 安装并统一收纳在 D:\tools\agent-tools 下
REM  (toolchain-atmelavr / tool-avrdude). 版本子目录自动解析。
REM =====================================================================

set "AGENT_TOOLS=D:\tools\agent-tools"
set "PKGS=%AGENT_TOOLS%\platformio-core\packages"

REM ---- 定位工具链 / avrdude (版本子目录自动解析) ----
set "TC="
for /d %%d in ("%PKGS%\toolchain-atmelavr*") do set "TC=%%d"
set "AVRDUDE_DIR="
for /d %%d in ("%PKGS%\tool-avrdude*") do set "AVRDUDE_DIR=%%d"

if not defined TC ( echo [ERROR] 未找到 toolchain-atmelavr (请先运行 pio pkg install --global --platform platformio/atmelavr) & exit /b 1 )
if not defined AVRDUDE_DIR ( echo [ERROR] 未找到 tool-avrdude & exit /b 1 )

set "CC=%TC%\bin\avr-gcc.exe"
set "CXX=%TC%\bin\avr-g++.exe"
set "OBJCOPY=%TC%\bin\avr-objcopy.exe"
set "SIZE=%TC%\bin\avr-size.exe"
set "AVRDUDE=%AVRDUDE_DIR%\bin\avrdude.exe"
set "AVRDUDE_CONF=%AVRDUDE_DIR%\avrdude.conf"

set "MCU=atmega328p"
set "FCPU=16000000"
set "PROJ=%~dp0.."
cd /d "%PROJ%"
set "OUT=%PROJ%\.build"
if not exist "%OUT%" mkdir "%OUT%"

set "CFLAGS=-mmcu=%MCU% -DF_CPU=%FCPU%UL -Os -Wall -Wextra -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -I. -Ibsp -Icore"
set "CXXFLAGS=%CFLAGS% -fno-exceptions -fno-rtti"

echo [BUILD] toolchain: %TC%
echo [BUILD] project:   %PROJ%

REM ---- 编译所有 .c 与 main.cpp ----
set "OBJS="
for %%f in (bsp\*.c core\*.c) do (
  set "o=%OUT%\%%~nf.o"
  echo   CC  %%f
  "%CC%" %CFLAGS% -c "%%f" -o "%OUT%\%%~nf.o"
  if errorlevel 1 exit /b 1
  set "OBJS=!OBJS! %OUT%\%%~nf.o"
)
echo   CXX core\main.cpp
"%CXX%" %CXXFLAGS% -c "core\main.cpp" -o "%OUT%\main.o"
if errorlevel 1 exit /b 1
set "OBJS=!OBJS! %OUT%\main.o"

REM ---- 链接 (avr-gcc 自动带 crt/avr-libc/libgcc) ----
echo [LINK] firmware.elf
"%CC%" -mmcu=%MCU% !OBJS! -o "%OUT%\firmware.elf" -Wl,-Map="%OUT%\firmware.map" -lm
if errorlevel 1 exit /b 1

REM ---- 生成 hex ----
"%OBJCOPY%" -O ihex -R .eeprom "%OUT%\firmware.elf" "%OUT%\firmware.hex"
if errorlevel 1 exit /b 1

REM ---- 体积统计 (FLASH=.text+.data, RAM=.data+.bss) ----
echo [SIZE]
for /f "tokens=1,2" %%a in ('"%SIZE%" -A "%OUT%\firmware.elf" ^| findstr /C:".text" /C:".data" /C:".bss"') do (
  if "%%a"==".text" set /a TEXT=%%b
  if "%%a"==".data" set /a DATA=%%b
  if "%%a"==".bss"  set /a BSS=%%b
)
set /a FLASH=TEXT+DATA
set /a RAM=DATA+BSS
echo   FLASH = %FLASH% B  (text %TEXT% + data %DATA%) / 32256 B  (atmega328p usable)
echo   RAM   = %RAM% B  (data %DATA% + bss %BSS%)  / 2048 B
echo [OK] build finished: %OUT%\firmware.hex
exit /b 0
