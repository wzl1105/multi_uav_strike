@echo off & chcp 65001 >nul
setlocal enabledelayedexpansion
cd /d "%~dp0"

set "TGT_IP=%~1"
if "%~1"=="" (
    set /p "TGT_IP=Target IP: "
    if not defined TGT_IP exit /b 1
)

set "SSH_USER=root"
set "TGT_BASE=/home/rpdzkj/tzs_ws"
set "SERVICES=tzs_ws-multi tzs_ws-payload"

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

if exist "!BUNDLE_ROOT!\start_gimbal.sh" (
    scp %SSH_OPTS% "!BUNDLE_ROOT!\start_gimbal.sh" %SSH_USER%@%TGT_IP%:%TGT_BASE%/start_gimbal.sh
    ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "chmod +x %TGT_BASE%/start_gimbal.sh"
)

ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "sudo systemctl daemon-reload && sudo systemctl restart %SERVICES%"

rmdir /S /Q "!WORKDIR!" 2>nul

pause
endlocal