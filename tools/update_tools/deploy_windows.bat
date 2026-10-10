@echo off & chcp 65001 >nul
setlocal enabledelayedexpansion
cd /d "%~dp0"

REM ============================================================
REM Interactive flow (double-click):
REM   1. Target IP: [192.168.1.151]    (default shown, Enter to accept)
REM   2. Action? [D=deploy / Q=query]   (D default, Enter to deploy)
REM
REM Command-line forms (skip prompts):
REM   deploy_windows.bat [target_ip]              deploy, prompt for action if no IP
REM   deploy_windows.bat query [target_ip]        query only
REM ============================================================

set "ACTION="
set "ARG_IP="

REM parse args
if /i "%~1"=="query" (
    set "ACTION=query"
    set "ARG_IP=%~2"
) else if /i "%~1"=="deploy" (
    set "ACTION=deploy"
    set "ARG_IP=%~2"
) else (
    set "ARG_IP=%~1"
)

set "TGT_IP=%ARG_IP%"
if "%TGT_IP%"=="" (
    set /p "TGT_IP=Target IP [192.168.1.151]: "
    if not defined TGT_IP set "TGT_IP=192.168.1.151"
)

if not defined ACTION goto :ASK_ACTION
goto :ARGS_DONE

:ASK_ACTION
echo.
echo What do you want to do on %TGT_IP% ?
set /p "ACTION_CHOICE=  [D]eploy (default)  /  [Q]uery version: "
if /i "%ACTION_CHOICE%"=="Q" (
    set "ACTION=query"
) else (
    set "ACTION=deploy"
)

:ARGS_DONE
set "SSH_USER=root"
set "TGT_BASE=/home/rpdzkj/tzs_ws"
set "SERVICES=tzs_ws-multi tzs_ws-payload"
set "PKG_NAME=multi_uav_strike"
REM package.xml is synced by deploy into src/<pkg>/; that's the source of truth
REM on the board (devel/share/<pkg>/package.xml only exists after a local build).
set "PKG_XML=%TGT_BASE%/src/%PKG_NAME%/package.xml"
set "LOCAL_XML=%~dp0..\..\package.xml"

if /i "%ACTION%"=="query" goto :DO_QUERY

where tar >nul 2>&1 || (echo ERROR: tar.exe not found & pause & exit /b 1)
where ssh >nul 2>&1 || (echo ERROR: ssh.exe not found & pause & exit /b 1)
where scp >nul 2>&1 || (echo ERROR: scp.exe not found & pause & exit /b 1)

set "BUNDLE="
for /f "delims=" %%F in ('dir /b /o-d tzs_ws_bundle_*.tar.gz 2^>nul') do if not defined BUNDLE set "BUNDLE=%%F"
if not defined BUNDLE (echo ERROR: no bundle found & pause & exit /b 1)

set "WORKDIR=%TEMP%\tzs_ws_deploy_%RANDOM%"
mkdir "!WORKDIR!" 2>nul
tar -xzf "!BUNDLE!" -C "!WORKDIR!" || (rmdir /S /Q "!WORKDIR!" 2>nul & echo ERROR: tar extract failed & pause & exit /b 1)

set "BUNDLE_ROOT="
for /d %%D in ("!WORKDIR!\*") do if not defined BUNDLE_ROOT set "BUNDLE_ROOT=%%D"
if not defined BUNDLE_ROOT (rmdir /S /Q "!WORKDIR!" 2>nul & echo ERROR: bundle root not found & pause & exit /b 1)

set "SSH_OPTS=-o StrictHostKeyChecking=no -o UserKnownHostsFile=nul"

echo.
echo ============================================================
echo Deploying to %TGT_IP% (each ssh/scp may prompt for password)
echo Press Ctrl+C to cancel
echo ============================================================
echo.

ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "true" || (rmdir /S /Q "!WORKDIR!" 2>nul & echo ERROR: SSH connect failed & pause & exit /b 1)

ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "sudo systemctl stop %SERVICES% 2>/dev/null || true"

for /d %%P in ("!BUNDLE_ROOT!\devel\lib\*") do (
    if /i not "%%~nxP"=="python3" scp %SSH_OPTS% -r "%%P\*" %SSH_USER%@%TGT_IP%:%TGT_BASE%/devel/lib/%%~nxP/
)

if exist "!BUNDLE_ROOT!\devel\lib\python3\dist-packages" (
    for /d %%Q in ("!BUNDLE_ROOT!\devel\lib\python3\dist-packages\*") do scp %SSH_OPTS% -r "%%Q\*" %SSH_USER%@%TGT_IP%:%TGT_BASE%/devel/lib/python3/dist-packages/%%~nxQ/
)

if exist "!BUNDLE_ROOT!\src" (
    for /d %%S in ("!BUNDLE_ROOT!\src\*") do scp %SSH_OPTS% -r "%%S\*" %SSH_USER%@%TGT_IP%:%TGT_BASE%/src/%%~nxS/
)

REM Fix permissions: scp as root leaves dirs as drwx------, blocks rpdzkj
REM (systemd User) from entering. Force all dirs under src/ and devel/lib/ to 755.
ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "find %TGT_BASE%/src %TGT_BASE%/devel/lib -type d -exec chmod 755 {} \;"

if exist "!BUNDLE_ROOT!\start_gimbal.sh" (
    scp %SSH_OPTS% "!BUNDLE_ROOT!\start_gimbal.sh" %SSH_USER%@%TGT_IP%:%TGT_BASE%/start_gimbal.sh
    ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "chmod +x %TGT_BASE%/start_gimbal.sh"
)

ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "sudo systemctl daemon-reload && sudo systemctl restart %SERVICES%"

rmdir /S /Q "!WORKDIR!" 2>nul

pause
endlocal
exit /b 0

REM ============================================================
REM :DO_QUERY  -  query package version on target board
REM Prints: target IP / remote version / local dev version (if found)
REM ============================================================
:DO_QUERY
where ssh >nul 2>&1 || (echo ERROR: ssh.exe not found & pause & exit /b 1)

set "SSH_OPTS=-o StrictHostKeyChecking=no -o UserKnownHostsFile=nul -o ConnectTimeout=5 -o LogLevel=ERROR"

echo.
echo ============================================================
echo Querying %PKG_NAME% version on %SSH_USER%@%TGT_IP%
echo ============================================================

REM Probe SSH first
ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "true" >nul 2>&1
if errorlevel 1 (
    echo ERROR: cannot SSH to %SSH_USER%@%TGT_IP%
    pause
    exit /b 1
)

REM --- remote version: read <version> from src/<pkg>/package.xml ---
REM Use sed (universally available on Rockchip boards) to extract the value.
REM Fallback chain: src/ first, then devel/share/ if src/ missing.
set "REMOTE_VER_FOUND="
for /f "usebackq delims=" %%V in (`ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "sed -n 's:^  *<version>\(.*\)</version>:\1:p' '%PKG_XML%' 2>/dev/null || sed -n 's:^  *<version>\(.*\)</version>:\1:p' '%TGT_BASE%/devel/share/%PKG_NAME%/package.xml' 2>/dev/null"`) do (
    if not "%%V"=="" (
        set "REMOTE_VER=%%V"
        set "REMOTE_VER_FOUND=1"
    )
)
if not defined REMOTE_VER_FOUND (
    REM Distinguish "file missing entirely" from "tag present but empty"
    ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "test -f '%PKG_XML%' -o -f '%TGT_BASE%/devel/share/%PKG_NAME%/package.xml'" >nul 2>&1
    if errorlevel 1 set "REMOTE_VER=NOT_FOUND"
)

REM --- local dev version (best-effort, for compare) ---
REM Use findstr with literal string (no regex) and a strict substring to
REM avoid cmd's ^< / ^> escape rules in /R mode.
set "LOCAL_VER="
if exist "%LOCAL_XML%" (
    for /f "usebackq delims=" %%V in (`findstr /C:"<version>" "%LOCAL_XML%"`) do (
        for /f "tokens=2 delims=<>" %%P in ("%%V") do (
            if not "%%P"=="" set "LOCAL_VER=%%P"
        )
    )
)

echo.
if defined REMOTE_VER_FOUND goto :PRINT_REMOTE
if /i "%REMOTE_VER%"=="NOT_FOUND" goto :REMOTE_MISSING
echo Board version  : NOT FOUND ^<version^> tag missing in package.xml
goto :REMOTE_DONE
:REMOTE_MISSING
echo Board version  : NOT FOUND
echo                 checked: %PKG_XML%
echo                          %TGT_BASE%/devel/share/%PKG_NAME%/package.xml
goto :REMOTE_DONE
:PRINT_REMOTE
echo Board version  : %REMOTE_VER%
:REMOTE_DONE

if defined LOCAL_VER (
    echo Local dev ver  : %LOCAL_VER%
) else (
    echo Local dev ver  : ^<not found at %LOCAL_XML%^>
)

echo.
echo Inspection path : %SSH_USER%@%TGT_IP%:%PKG_XML%
echo ============================================================
pause
endlocal
exit /b 0