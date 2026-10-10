@echo off & chcp 65001 >nul
setlocal enabledelayedexpansion
cd /d "%~dp0"

REM ============================================================
REM LN_config.bat — 板子运维入口（Windows 双击或命令行）
REM
REM 菜单:
REM   [D]eploy   覆盖 bundle 到板子, restart 服务
REM   [Q]uery    只查板子 / 本地 package.xml 版本
REM   [G]rab     停板子 rosbag record (rosnode kill), 拉最新 group 的 .bag(.active) 回 bat 运行目录
REM   [T]rigger  手动触发拉烟 (ssh 到板子发 rostopic pub /yx/cmd/trigger_smoke), 发前 y/N 确认
REM
REM Command-line forms (skip prompts):
REM   LN_config.bat                          菜单 (问 IP → 问动作)
REM   LN_config.bat <target_ip>              菜单 (IP 预填)
REM   LN_config.bat query [target_ip]        query only
REM   LN_config.bat deploy <ip> [bundle]     deploy only (skip menu)
REM   LN_config.bat grab [target_ip]         grab latest bag
REM   LN_config.bat trigger [target_ip]      trigger smoke (will prompt y/N inside)
REM ============================================================

set "ACTION="
set "ARG_IP="
set "ARG_BUNDLE="

REM parse args
if /i "%~1"=="query" (
    set "ACTION=query"
    set "ARG_IP=%~2"
) else if /i "%~1"=="deploy" (
    set "ACTION=deploy"
    set "ARG_IP=%~2"
    set "ARG_BUNDLE=%~3"
) else if /i "%~1"=="grab" (
    set "ACTION=grab"
    set "ARG_IP=%~2"
) else if /i "%~1"=="trigger" (
    set "ACTION=trigger"
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
set /p "ACTION_CHOICE=  [D]eploy (default)  /  [Q]uery version  /  [G]rab latest bag  /  [T]rigger smoke: "
if /i "%ACTION_CHOICE%"=="Q" set "ACTION=query"
if /i "%ACTION_CHOICE%"=="G" set "ACTION=grab"
if /i "%ACTION_CHOICE%"=="T" set "ACTION=trigger"
if not defined ACTION set "ACTION=deploy"

:ARGS_DONE
set "SSH_USER=root"
set "TGT_BASE=/home/rpdzkj/tzs_ws"
set "BAG_DIR=/home/rpdzkj/tzs_ws/LOG"
set "SERVICES=tzs_ws-multi tzs_ws-payload"
set "PKG_NAME=multi_uav_strike"
REM package.xml is synced by deploy into src/<pkg>/; that's the source of truth
REM on the board (devel/share/<pkg>/package.xml only exists after a local build).
set "PKG_XML=%TGT_BASE%/src/%PKG_NAME%/package.xml"
set "DEVEL_XML=%TGT_BASE%/devel/share/%PKG_NAME%/package.xml"
set "LOCAL_XML=%~dp0..\..\package.xml"
set "SSH_OPTS=-o StrictHostKeyChecking=no -o UserKnownHostsFile=nul -o ConnectTimeout=5 -o LogLevel=ERROR"

if /i "%ACTION%"=="query" goto :DO_QUERY
if /i "%ACTION%"=="grab" goto :DO_GRAB
if /i "%ACTION%"=="trigger" goto :DO_TRIGGER
goto :DO_DEPLOY

REM ============================================================
REM :DO_QUERY  -  query package version on target board
REM ============================================================
:DO_QUERY
where ssh >nul 2>&1 || (echo ERROR: ssh.exe not found & pause & exit /b 1)

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
set "REMOTE_VER_FOUND="
for /f "usebackq delims=" %%V in (`ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "sed -n 's:^  *<version>\(.*\)</version>:\1:p' '%PKG_XML%' 2>/dev/null || sed -n 's:^  *<version>\(.*\)</version>:\1:p' '%DEVEL_XML%' 2>/dev/null"`) do (
    if not "%%V"=="" (
        set "REMOTE_VER=%%V"
        set "REMOTE_VER_FOUND=1"
    )
)
if not defined REMOTE_VER_FOUND (
    ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "test -f '%PKG_XML%' -o -f '%DEVEL_XML%'" >nul 2>&1
    if errorlevel 1 set "REMOTE_VER=NOT_FOUND"
)

REM --- local dev version (best-effort, for compare) ---
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
echo                          %DEVEL_XML%
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

REM ============================================================
REM :DO_GRAB  -  rosnode kill 板子 rosbag, scp 拉最新 group 的 .bag 回 bat 当前目录
REM   bat 启动时已经 cd /d %~dp0 切到脚本所在目录, 直接落 %cd%。
REM   严禁删板子 bag (跟 feedback_dont_delete_log_files.md 一致)。
REM ============================================================
:DO_GRAB
where ssh >nul 2>&1 || (echo ERROR: ssh.exe not found & pause & exit /b 1)
where scp >nul 2>&1 || (echo ERROR: scp.exe not found & pause & exit /b 1)

echo.
echo ============================================================
echo Grabbing latest bag from %SSH_USER%@%TGT_IP%:%BAG_DIR%
echo ============================================================

REM Probe SSH first
ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "true" >nul 2>&1
if errorlevel 1 (
    echo ERROR: cannot SSH to %SSH_USER%@%TGT_IP%
    pause
    exit /b 1
)

REM 1) find /record_<N> node, kill if running
set "RECORD_NODE="
for /f "usebackq delims=" %%N in (`ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "rosnode list 2>/dev/null | grep '^/record_' | head -1"`) do (
    set "RECORD_NODE=%%N"
)
if defined RECORD_NODE (
    echo [grab] killing rosbag node: %RECORD_NODE%
    ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "rosnode kill '%RECORD_NODE%' 2>&1" || true
    REM 等待 finalize (.active -> .bag)
    timeout /t 3 /nobreak >nul
) else (
    echo [grab] no /record_* node running, skipping kill
)

REM 2) list bag files, 取 N 最大那一组
REM    文件名: multi_uav_flight_<N>_<M>.bag(.active)
REM    板子端没有 awk 也不一定, 用 for 循环在 forfiles / ls 输出里切。
set "GROUP_FILES_TMP=%TEMP%\tzs_grab_files_%RANDOM%.txt"
ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "ls -1 '%BAG_DIR%'/multi_uav_flight_*.bag* 2>/dev/null" > "%GROUP_FILES_TMP%" 2>nul
if errorlevel 1 (
    echo ERROR: ssh ls failed
    del "%GROUP_FILES_TMP%" 2>nul
    pause
    exit /b 1
)

REM 找最大 N (用 powershell, cmd 没浮点解析, 一行搞定)
set "MAX_N=0"
for /f "usebackq delims=" %%M in (`powershell -NoProfile -Command "$lines = Get-Content '%GROUP_FILES_TMP%'; $max = 0; foreach ($l in $lines) { $bn = Split-Path $l -Leaf; $parts = $bn -split '_'; if ($parts.Length -ge 4) { $n = 0; if ([int]::TryParse($parts[3], [ref]$n)) { if ($n -gt $max) { $max = $n } } } }; $max"`) do (
    set "MAX_N=%%M"
)
if "%MAX_N%"=="0" (
    echo ERROR: 解析 bag 文件名失败, 样本:
    type "%GROUP_FILES_TMP%"
    del "%GROUP_FILES_TMP%" 2>nul
    pause
    exit /b 1
)

REM 过滤出 N == MAX_N 的所有行
set "GROUP_FILES=%TEMP%\tzs_grab_group_%RANDOM%.txt"
powershell -NoProfile -Command "$lines = Get-Content '%GROUP_FILES_TMP%'; $target = %MAX_N%; $lines | Where-Object { $bn = Split-Path $_ -Leaf; $parts = $bn -split '_'; $n = 0; if ([int]::TryParse($parts[3], [ref]$n)) { $n -eq $target } }" > "%GROUP_FILES%" 2>nul

echo [grab] latest group N=%MAX_N%, files:
type "%GROUP_FILES%" | findstr /R "." >nul && type "%GROUP_FILES%" || (
    echo ERROR: 过滤后没有匹配 N=%MAX_N% 的文件
    del "%GROUP_FILES_TMP%" "%GROUP_FILES%" 2>nul
    pause
    exit /b 1
)

REM 3) scp 拉回 bat 当前目录
echo [grab] scp -^> %cd%
for /f "usebackq delims=" %%R in ("%GROUP_FILES%") do (
    scp %SSH_OPTS% %SSH_USER%@%TGT_IP%:"%%R" "%cd%\\" || (
        echo ERROR: scp failed for %%R
        del "%GROUP_FILES_TMP%" "%GROUP_FILES%" 2>nul
        pause
        exit /b 1
    )
)

echo.
echo === GRAB DONE ===
echo Local dir: %cd%
dir /b multi_uav_flight_%MAX_N%_* 2>nul
del "%GROUP_FILES_TMP%" "%GROUP_FILES%" 2>nul
pause
endlocal
exit /b 0

REM ============================================================
REM :DO_TRIGGER  -  ssh 到板子发 rostopic pub 拉烟, 发前 y/N 确认
REM   armed gate 由板子端 guidance_control 自行决定 (yaml: manual_smoke_bypass_armed_gate)
REM ============================================================
:DO_TRIGGER
where ssh >nul 2>&1 || (echo ERROR: ssh.exe not found & pause & exit /b 1)

echo.
echo ============================================================
echo TRIGGER SMOKE on %SSH_USER%@%TGT_IP%
echo   rostopic pub /yx/cmd/trigger_smoke std_msgs/Bool "data: true" -1
echo   (板子端 armed gate 由 guidance_control manual_smoke_bypass_armed_gate 决定)
echo ============================================================
set /p "CONFIRM=Are you sure you want to trigger smoke on %TGT_IP% ? [y/N] "
if /i not "%CONFIRM%"=="y" if /i not "%CONFIRM%"=="Y" if /i not "%CONFIRM%"=="yes" if /i not "%CONFIRM%"=="YES" (
    echo [trigger] cancelled by user
    pause
    endlocal
    exit /b 0
)
echo [trigger] confirmed, sending...
ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "bash -lc 'source /opt/ros/*/setup.bash 2>/dev/null; rostopic pub -1 /yx/cmd/trigger_smoke std_msgs/Bool \"data: true\" 2>&1'" || (
    echo ERROR: 触发失败, 板子端 rostopic pub 报错 (见上)
    pause
    endlocal
    exit /b 1
)
echo === TRIGGER DONE ===
pause
endlocal
exit /b 0

REM ============================================================
REM :DO_DEPLOY  -  default action
REM ============================================================
:DO_DEPLOY
where tar >nul 2>&1 || (echo ERROR: tar.exe not found & pause & exit /b 1)
where ssh >nul 2>&1 || (echo ERROR: ssh.exe not found & pause & exit /b 1)
where scp >nul 2>&1 || (echo ERROR: scp.exe not found & pause & exit /b 1)

set "BUNDLE=%ARG_BUNDLE%"
if "%BUNDLE%"=="" (
    for /f "delims=" %%F in ('dir /b /o-d tzs_ws_bundle_*.tar.gz 2^>nul') do if not defined BUNDLE set "BUNDLE=%%F"
)
if not defined BUNDLE (echo ERROR: no bundle found & pause & exit /b 1)

set "WORKDIR=%TEMP%\tzs_ws_deploy_%RANDOM%"
mkdir "!WORKDIR!" 2>nul
tar -xzf "!BUNDLE!" -C "!WORKDIR!" || (rmdir /S /Q "!WORKDIR!" 2>nul & echo ERROR: tar extract failed & pause & exit /b 1)

set "BUNDLE_ROOT="
for /d %%D in ("!WORKDIR!\*") do if not defined BUNDLE_ROOT set "BUNDLE_ROOT=%%D"
if not defined BUNDLE_ROOT (rmdir /S /Q "!WORKDIR!" 2>nul & echo ERROR: bundle root not found & pause & exit /b 1)

set "SSH_OPTS_DEPLOY=-o StrictHostKeyChecking=no -o UserKnownHostsFile=nul"

echo.
echo ============================================================
echo Deploying to %TGT_IP% (each ssh/scp may prompt for password)
echo Press Ctrl+C to cancel
echo ============================================================
echo.

ssh %SSH_OPTS_DEPLOY% %SSH_USER%@%TGT_IP% "true" || (rmdir /S /Q "!WORKDIR!" 2>nul & echo ERROR: SSH connect failed & pause & exit /b 1)

ssh %SSH_OPTS_DEPLOY% %SSH_USER%@%TGT_IP% "sudo systemctl stop %SERVICES% 2>/dev/null || true"

for /d %%P in ("!BUNDLE_ROOT!\devel\lib\*") do (
    if /i not "%%~nxP"=="python3" scp %SSH_OPTS_DEPLOY% -r "%%P\*" %SSH_USER%@%TGT_IP%:%TGT_BASE%/devel/lib/%%~nxP/
)

if exist "!BUNDLE_ROOT!\devel\lib\python3\dist-packages" (
    for /d %%Q in ("!BUNDLE_ROOT!\devel\lib\python3\dist-packages\*") do scp %SSH_OPTS_DEPLOY% -r "%%Q\*" %SSH_USER%@%TGT_IP%:%TGT_BASE%/devel/lib/python3/dist-packages/%%~nxQ/
)

if exist "!BUNDLE_ROOT!\src" (
    for /d %%S in ("!BUNDLE_ROOT!\src\*") do scp %SSH_OPTS_DEPLOY% -r "%%S\*" %SSH_USER%@%TGT_IP%:%TGT_BASE%/src/%%~nxS/
)

REM Fix permissions: scp as root leaves dirs as drwx------, blocks rpdzkj
REM (systemd User) from entering. Force all dirs under src/ and devel/lib/ to 755.
ssh %SSH_OPTS_DEPLOY% %SSH_USER%@%TGT_IP% "find %TGT_BASE%/src %TGT_BASE%/devel/lib -type d -exec chmod 755 {} \;"

if exist "!BUNDLE_ROOT!\start_gimbal.sh" (
    scp %SSH_OPTS_DEPLOY% "!BUNDLE_ROOT!\start_gimbal.sh" %SSH_USER%@%TGT_IP%:%TGT_BASE%/start_gimbal.sh
    ssh %SSH_OPTS_DEPLOY% %SSH_USER%@%TGT_IP% "chmod +x %TGT_BASE%/start_gimbal.sh"
)

ssh %SSH_OPTS_DEPLOY% %SSH_USER%@%TGT_IP% "sudo systemctl daemon-reload && sudo systemctl restart %SERVICES%"

rmdir /S /Q "!WORKDIR!" 2>nul

pause
endlocal
exit /b 0
