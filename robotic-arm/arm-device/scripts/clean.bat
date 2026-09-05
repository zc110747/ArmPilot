@echo off
setlocal EnableExtensions
REM remove build artifacts
set "PROJ=%~dp0.."
if exist "%PROJ%\.build" rmdir /s /q "%PROJ%\.build"
echo [CLEAN] removed .build
exit /b 0
