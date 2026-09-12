@echo off
rem ============================================================================
rem  ArmPilot MeArm-3D -- one-click launcher
rem ----------------------------------------------------------------------------
rem  Starts the Go backend and the Vite frontend, each in its own console window.
rem
rem  Usage:
rem      start.bat          SIM mode (default, does NOT touch hardware)
rem      start.bat --real   REAL mode (config.serial.yaml, moves the physical arm)
rem      start.bat --help   show this help
rem
rem  Design notes
rem  ------------
rem  * This file is intentionally pure ASCII. Chinese comments break the Windows
rem    GBK console parser and have bitten us before.
rem  * Model / calibration / limit truth lives ONLY in config\robot.yaml.
rem    This script never duplicates, rewrites or overrides any of it.
rem  * REAL mode drives physical servos. The arm WILL move.
rem  * The serial port is read from backend\config.serial.yaml as-is. The script
rem    deliberately does NOT rewrite YAML: doing so from a .bat mangles UTF-8
rem    comments and CRLF line endings, and a half-parsed config is far more
rem    dangerous than a clear "port not found" from the backend.
rem  * A bare ">" inside an echo line that sits in a parenthesised block is parsed
rem    as redirection for the WHOLE block and silently kills it. Every literal ">"
rem    printed as text is therefore caret-escaped as "^>".
rem ============================================================================
setlocal EnableExtensions EnableDelayedExpansion

rem ---- Locate this script's directory ----------------------------------------
rem  %~dp0 ends with a backslash; pushd handles the trailing separator fine.
set "ROOT=%~dp0"
pushd "%ROOT%" || (echo [FAIL] Cannot enter script directory: %ROOT% & goto :end)

set "BACKEND_DIR=%ROOT%backend"
set "FRONTEND_DIR=%ROOT%frontend"
set "BACKEND_EXE=%BACKEND_DIR%\bin\armpilot-backend.exe"
set "WEB_PORT=8090"
set "FE_PORT=5273"

set "MODE=sim"
set "CFG=config.yaml"

rem ---- Parse arguments -------------------------------------------------------
:parse
if "%~1"=="" goto :parsed
if /i "%~1"=="--help" goto :usage
if /i "%~1"=="-h"     goto :usage
if /i "%~1"=="/?"     goto :usage
if /i "%~1"=="--real" (set "MODE=real" & set "CFG=config.serial.yaml" & shift & goto :parse)
if /i "%~1"=="--sim"  (set "MODE=sim"  & set "CFG=config.yaml"        & shift & goto :parse)
echo [FAIL] Unknown argument: %~1
echo        Run "start.bat --help" for usage.
goto :end

:parsed
set "CFG_PATH=%BACKEND_DIR%\%CFG%"

echo.
echo ===========================================================================
echo  ArmPilot MeArm-3D launcher
echo ===========================================================================
echo  Mode      : %MODE%
echo  Backend   : %CFG%
echo.

rem ---- Preflight: required files --------------------------------------------
if not exist "%BACKEND_EXE%" (
  echo [FAIL] Backend binary missing:
  echo        %BACKEND_EXE%
  echo        Build it first:  cd backend ^&^& go build -o bin\armpilot-backend.exe .
  goto :end
)
if not exist "%CFG_PATH%" (
  echo [FAIL] Config file missing:
  echo        %CFG_PATH%
  goto :end
)
if not exist "%ROOT%config\robot.yaml" (
  echo [FAIL] Model truth file missing: config\robot.yaml
  echo        Refusing to start: hardcoded fallbacks would silently diverge.
  goto :end
)

where node >nul 2>nul
if errorlevel 1 (
  echo [FAIL] node not found on PATH. Install Node.js 20 or newer and retry.
  goto :end
)

rem  Prefer the locally pinned vite launcher: it guarantees the locked version and
rem  still works when a managed runtime shadows the global npm shim.
set "VITE_CMD=%FRONTEND_DIR%\node_modules\.bin\vite.cmd"
if exist "%VITE_CMD%" goto :got_vite
where npm >nul 2>nul
if errorlevel 1 (
  echo [FAIL] Neither frontend\node_modules\.bin\vite.cmd nor npm is available.
  echo        Run:  cd frontend ^&^& npm install
  goto :end
)

:got_vite

rem ---- Port preflight --------------------------------------------------------
echo [1/3] Checking ports %WEB_PORT% (backend) and %FE_PORT% (frontend) ...
set "BUSY="
for /f "tokens=5" %%P in ('netstat -ano -p TCP ^| findstr /r /c:":%WEB_PORT% .*LISTENING"') do (
  if not "%%P"=="0" set "BUSY=!BUSY! %WEB_PORT%:%%P"
)
for /f "tokens=5" %%P in ('netstat -ano -p TCP ^| findstr /r /c:":%FE_PORT% .*LISTENING"') do (
  if not "%%P"=="0" set "BUSY=!BUSY! %FE_PORT%:%%P"
)

if not defined BUSY (
  echo       Both ports are free.
  goto :after_ports
)

echo.
echo       [WARN] Stale listener^(s^) detected:
for %%B in (!BUSY!) do (
  for /f "tokens=1,2 delims=:" %%X in ("%%B") do (
    echo         port %%X  P-ID %%Y
    tasklist /fi "PID eq %%Y" /fo table /nh 2>nul
  )
)
echo.
echo       A previous run probably did not exit cleanly. If these are ArmPilot
echo       processes they can be terminated now; terminating the wrong process
echo       is possible, so this asks first.
echo.
set "ANSWER="
set /p "ANSWER=      Terminate these processes? [y/N] "
if /i not "!ANSWER!"=="y" (
  echo       Skipped. The backend or frontend may fail to bind -- check manually.
  goto :after_ports
)
for %%B in (!BUSY!) do (
  for /f "tokens=1,2 delims=:" %%X in ("%%B") do (
    echo         terminating P-ID %%Y ...
    taskkill /PID %%Y /F >nul 2>nul
  )
)
echo       Done.

:after_ports

rem ---- Resolve LAN address ---------------------------------------------------
set "LAN_IP="
for /f "tokens=1,2 delims=:" %%A in ('ipconfig ^| findstr /c:"IPv4"') do (
  if not defined LAN_IP (
    set "CAND=%%B"
    set "CAND=!CAND: =!"
    echo !CAND!| findstr /r /c:"^192\.168\." /c:"^10\." >nul
    if not errorlevel 1 set "LAN_IP=!CAND!"
  )
)

rem ---- Launch ----------------------------------------------------------------
rem  Quote pattern matters here. "cd /d "X" && prog" does NOT work: cmd strips the
rem  outer quotes, treats the first token as a directory, then tries to run a
rem  command literally named "then". Passing /d and the program as separate
rem  arguments to start avoids the whole problem.
echo.
echo [2/3] Launching services ...
if /i "%MODE%"=="real" (
  echo.
  echo  +------------------------------------------------------------------+
  echo  ^|  REAL MODE -- this WILL move the physical arm.                   ^|
  echo  ^|  Config: config.serial.yaml                                      ^|
  echo  ^|  Close each window, or press Ctrl+C, to stop it.                 ^|
  echo  +------------------------------------------------------------------+
)

start "ArmPilot backend [%MODE%]" /d "%BACKEND_DIR%" "%BACKEND_EXE%" -c "%CFG%"

rem  ---- Frontend: auto-connect over WebSocket --------------------------------
rem  The page otherwise boots in MockTransport (in-browser sim) and sits there
rem  until you click Connect -- which is exactly the "launcher ran, backend is
rem  up, but only sim data moves" trap. These env vars make the page connect
rem  itself to the backend we just started.
rem
rem    VITE_AUTO_CONNECT=ws   -> connect the WebSocket backend (not in-page mock)
rem    VITE_AUTO_REAL=1       -> also switch to Real Robot. Only meaningful in
rem                              REAL mode; in SIM mode the backend link end is
rem                              "sim", so the switch is refused with a visible
rem                              warning (the arm is NOT touched either way).
rem
rem  VITE_WS_URL is deliberately left unset: the frontend derives the host from
rem  window.location, so LAN clients (http://<ip>:5273) reach the right machine.
set "FE_AUTO_CONNECT=ws"
set "FE_AUTO_REAL="
if /i "%MODE%"=="real" set "FE_AUTO_REAL=1"

if exist "%VITE_CMD%" (
  start "ArmPilot frontend" /d "%FRONTEND_DIR%" cmd /k "set VITE_AUTO_CONNECT=%FE_AUTO_CONNECT%&& set VITE_AUTO_REAL=%FE_AUTO_REAL%&& "%VITE_CMD%""
) else (
  start "ArmPilot frontend" /d "%FRONTEND_DIR%" cmd /k "set VITE_AUTO_CONNECT=%FE_AUTO_CONNECT%&& set VITE_AUTO_REAL=%FE_AUTO_REAL%&& npm run dev"
)

echo [3/3] Done.
echo.
echo  ---------------------------------------------------------------------------
echo   Backend  : http://localhost:%WEB_PORT%/healthz
echo   Frontend : http://localhost:%FE_PORT%
if defined LAN_IP echo   LAN      : http://%LAN_IP%:%FE_PORT%
echo  ---------------------------------------------------------------------------
if /i "%MODE%"=="real" (
  echo.
  echo   REAL mode next steps:
  echo     1. Wait for the backend window to print: [serial] connected ...
  echo     2. In the web UI: Connection -^> WebSocket
  echo        URL: ws://localhost:%WEB_PORT%/ws/joint then click Connect
  echo     3. Click "Real Robot" and confirm the hint reads:
  echo        "* driving real arm (link end = serial)"
  echo.
  echo   If that hint does NOT appear, the link is wrong and the arm will not
  echo   move -- check steps 1 and 2 above.
) else (
  echo.
  echo   SIM mode: no hardware is touched. Commands only move the virtual arm.
  echo   To drive the real arm, close these windows and rerun: start.bat --real
)
echo.
echo   Truth file: config\robot.yaml ^(not modified by this script^)
echo.
echo   Press any key to close this launcher window.
echo   The two service windows keep running until you close them.
pause >nul
echo   Bye.
goto :end

:usage
echo.
echo  ArmPilot MeArm-3D launcher
echo.
echo    start.bat          SIM mode ^(default; safe, no hardware^)
echo    start.bat --real   REAL mode ^(config.serial.yaml; the arm moves^)
echo    start.bat --sim    force SIM mode
echo    start.bat --help   this message
echo.
echo  Ports: backend %WEB_PORT%, frontend %FE_PORT%
echo  Serial port: read from backend\config.serial.yaml, edit it there
echo  Truth file : config\robot.yaml ^(model / calibration / limits^)
goto :end

:end
popd
endlocal
exit /b 0
