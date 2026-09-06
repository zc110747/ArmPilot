@echo off
REM arm-web one-click build: go vet + go build (web assets embedded via go:embed).
REM Run from the arm-web directory. Produces arm-web.exe.
setlocal
cd /d "%~dp0."

set GOFLAGS=-mod=mod
set GOPROXY=off
set GOSUMDB=off

set OUT=arm-web.exe

echo [build] go vet ./...
go vet ./...
if errorlevel 1 (
    echo [build] FAILED: go vet
    exit /b 1
)

echo [build] go build -o %OUT% .
go build -o %OUT% .
if errorlevel 1 (
    echo [build] FAILED: go build
    exit /b 1
)

for %%F in (%OUT%) do echo [build] OK: %OUT% - %%~zF bytes
echo [build] done.
endlocal
