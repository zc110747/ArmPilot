@echo off
setlocal EnableExtensions
REM 清理构建产物
set "PROJ=%~dp0.."
if exist "%PROJ%\.build" rmdir /s /q "%PROJ%\.build"
echo [CLEAN] removed .build
exit /b 0
