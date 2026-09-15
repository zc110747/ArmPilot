@echo off
setlocal EnableExtensions
REM =====================================================================
REM  remove build artifacts (.build) for the meArm firmware
REM
REM  build.bat clean and start.bat clean are the primary paths - this file
REM  is the shortcut for wiping the tree without starting a compile.
REM
REM  PAUSE before the exit so a double-click keeps the message on screen.
REM  ASCII only - Chinese comments break the Windows GBK parser.
REM =====================================================================

call "%~dp0_env.bat"

if exist "%PROJ%\.build" rmdir /s /q "%PROJ%\.build"
echo.
echo [CLEAN] removed %PROJ%\.build
echo.
echo Press any key to close this window . . .
pause >nul
exit /b 0
