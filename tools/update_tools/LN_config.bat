@echo off & chcp 65001 >nul
setlocal enabledelayedexpansion
cd /d "%~dp0"

REM ============================================================
REM LN_config.bat -- board operations entry (Windows double-click or CLI)
REM
REM Menu:
REM   [D]eploy   push bundle to board + restart services
REM   [Q]uery    show board / local package.xml version
REM   [G]rab     stop board rosbag record (rosnode kill), scp latest group .bag(.active) back to bat cwd
REM   [T]rigger  manual smoke trigger (ssh to board, rostopic pub /yx/cmd/trigger_smoke), y/N confirm first
REM
REM Command-line forms (skip prompts):
REM   LN_config.bat                          menu (ask IP -> ask action)
REM   LN_config.bat <target_ip>              menu (IP prefilled)
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
set "SSH_MASTER_PATH=%TEMP%\ssh-master-lnconfig-%RANDOM%"
REM   No quotes around %SSH_MASTER_PATH% inside -o ControlPath: nested double
REM   quotes inside a set "..." statement get consumed by cmd.exe, leaving the
REM   control socket path malformed ("getsockname failed: Bad file descriptor").
set "SSH_OPTS=-o ControlMaster=auto -o ControlPath=%SSH_MASTER_PATH% -o ControlPersist=600 -o StrictHostKeyChecking=no -o UserKnownHostsFile=nul -o ConnectTimeout=5 -o LogLevel=ERROR"

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
REM   stderr stays visible (password prompt from ssh.exe); stdout suppressed
REM   because "true" outputs nothing. "2>&1" would also suppress the prompt.
ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "true" >nul
if errorlevel 1 (
    echo ERROR: cannot SSH to %SSH_USER%@%TGT_IP%
    pause
    exit /b 1
)

REM --- remote version: SSH only cats the file, parse <version> locally to avoid
REM     sed/awk escape hell in cmd.exe (ssh "command-line line 0: missing argument").
set "REMOTE_VER_FOUND="
set "TMP_PKG_XML=%TEMP%\tzs_pkg_xml_%RANDOM%.txt"
ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "cat '%PKG_XML%' 2>/dev/null || cat '%DEVEL_XML%' 2>/dev/null" > "%TMP_PKG_XML%" 2>nul
echo.
if exist "%TMP_PKG_XML%" (
    REM tokens=3 delims=<> picks the content BETWEEN the tags: <version>X</version>
    REM tokens=2 would just give the literal "version" inside the opening tag.
    for /f "usebackq tokens=3 delims=<>" %%P in (`findstr /C:"<version>" "%TMP_PKG_XML%"`) do (
        if not defined REMOTE_VER_FOUND (
            set "REMOTE_VER=%%P"
            set "REMOTE_VER_FOUND=1"
        )
    )
    del "%TMP_PKG_XML%" 2>nul
)
if not defined REMOTE_VER_FOUND (
    REM Same as probe above: keep stderr visible (password prompt) if master
    REM is somehow not yet established; stdout suppressed because "test -f" outputs nothing.
    ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "test -f '%PKG_XML%' -o -f '%DEVEL_XML%'" >nul
    if errorlevel 1 set "REMOTE_VER=NOT_FOUND"
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

echo.
echo Inspection path : %SSH_USER%@%TGT_IP%:%PKG_XML%
echo ============================================================
pause
endlocal
exit /b 0

REM ============================================================
REM :DO_GRAB  -  rosnode kill board rosbag, scp latest group .bag to bat cwd
REM   bat startup already did cd /d %~dp0 to script dir, files land in %cd%.
REM   NEVER delete board bags (see feedback_dont_delete_log_files.md).
REM ============================================================
:DO_GRAB
where ssh >nul 2>&1 || (echo ERROR: ssh.exe not found & pause & exit /b 1)
where scp >nul 2>&1 || (echo ERROR: scp.exe not found & pause & exit /b 1)

echo.
echo ============================================================
echo Grabbing latest bag from %SSH_USER%@%TGT_IP%:%BAG_DIR%
echo ============================================================

REM SSH ControlMaster: first ssh prompts for password, subsequent ssh/scp reuse the socket
REM   Required because each ssh call here would otherwise re-prompt for password
set "SSH_MASTER_PATH=%TEMP%\ssh-master-lnconfig-%RANDOM%"

REM SSH_OPTS (with ControlMaster) is set globally near the top of the file.
REM   Do NOT re-set it here: the previous G-only set was a leftover from when
REM   ControlMaster was per-segment, and it had nested-quote bugs that broke
REM   ControlPath ("getsockname failed: Bad file descriptor").
REM Probe SSH (also opens master)
REM   stdout suppressed (true outputs nothing), but stderr MUST stay visible
REM   so the password prompt from ssh.exe is shown to the user. With "2>&1"
REM   the prompt is also redirected to nul, user never sees it, ssh times out,
REM   then the cleanup "-O exit" below opens a fresh connection (no master
REM   was actually established) and the user lands in an interactive shell.
ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "true" >nul
if errorlevel 1 (
    echo ERROR: cannot SSH to %SSH_USER%@%TGT_IP%
    pause
    exit /b 1
)

REM 1) find rosbag record process by pgrep, get PID, send SIGINT (graceful, .active -> .bag)
REM    Avoid rosnode list (which needs ROS env and may hang on board if login shell stalls)
REM    SSH output redirected to a temp file, NOT captured by for /f backticks.
REM    Backtick+for+f capturing ssh's stderr (which can contain "Warning: ..."
REM    or pgrep error text) into a for /f parse has been seen to crash cmd.exe
REM    on Windows (window flashes shut). File-based read avoids that.
echo [grab] probing rosbag record process on board...
set "ROSBAG_PID="
set "PID_TMP=%TEMP%\tzs_rosbag_pid_%RANDOM%.txt"
ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "pgrep -f 'rosbag record' | head -1" > "%PID_TMP%" 2>nul
if exist "%PID_TMP%" (
    for /f "usebackq delims=" %%P in ("%PID_TMP%") do (
        if not defined ROSBAG_PID set "ROSBAG_PID=%%P"
    )
    del "%PID_TMP%" 2>nul
)
if defined ROSBAG_PID (
    echo [grab] detected rosbag record PID: %ROSBAG_PID%
    echo [grab] sending SIGINT (graceful, lets .active finalize to .bag)...
    ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "kill -INT %ROSBAG_PID% 2>&1" >nul
    echo [grab] waiting 4s for rosbag to finalize...
    timeout /t 4 /nobreak >nul
    echo [grab] finalize wait done
    REM ensure it's dead (busybox pgrep may not have -f; SIGINT usually enough)
    ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "kill -0 %ROSBAG_PID% 2>/dev/null && kill -KILL %ROSBAG_PID% 2>/dev/null" >nul
) else (
    echo [grab] no rosbag record process running, skipping kill
)

REM 2) list bag files, pick the group with the highest N
REM    filename: multi_uav_flight_<N>_<M>.bag(.active)
REM    board may not have awk, so use powershell on the local output.
echo [grab] listing bag files on board...
set "GROUP_FILES_TMP=%TEMP%\tzs_grab_files_%RANDOM%.txt"
ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "ls -1 '%BAG_DIR%'/multi_uav_flight_*.bag* 2>/dev/null" > "%GROUP_FILES_TMP%" 2>nul
echo [grab] list done
if errorlevel 1 (
    echo ERROR: ssh ls failed
    del "%GROUP_FILES_TMP%" 2>nul
    pause
    exit /b 1
)

REM Find max N (use powershell, cmd has no float parsing, one-liner)
set "MAX_N=0"
for /f "usebackq delims=" %%M in (`powershell -NoProfile -Command "$lines = Get-Content '%GROUP_FILES_TMP%'; $max = 0; foreach ($l in $lines) { $bn = Split-Path $l -Leaf; $parts = $bn -split '_'; if ($parts.Length -ge 4) { $n = 0; if ([int]::TryParse($parts[3], [ref]$n)) { if ($n -gt $max) { $max = $n } } } }; $max"`) do (
    set "MAX_N=%%M"
)
if "%MAX_N%"=="0" (
    echo ERROR: failed to parse bag filenames, sample:
    type "%GROUP_FILES_TMP%"
    del "%GROUP_FILES_TMP%" 2>nul
    pause
    exit /b 1
)

REM Filter to keep only lines where N == MAX_N
set "GROUP_FILES=%TEMP%\tzs_grab_group_%RANDOM%.txt"
powershell -NoProfile -Command "$lines = Get-Content '%GROUP_FILES_TMP%'; $target = %MAX_N%; $lines | Where-Object { $bn = Split-Path $_ -Leaf; $parts = $bn -split '_'; $n = 0; if ([int]::TryParse($parts[3], [ref]$n)) { $n -eq $target } }" > "%GROUP_FILES%" 2>nul

echo [grab] latest group N=%MAX_N%, files:
type "%GROUP_FILES%" | findstr /R "." >nul && type "%GROUP_FILES%" || (
    echo ERROR: no files matched N=%MAX_N% after filter
    del "%GROUP_FILES_TMP%" "%GROUP_FILES%" 2>nul
    pause
    exit /b 1
)

REM 3) scp back to bat cwd
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
echo.
echo [grab] DONE. Press any key to close this window...
pause
endlocal
exit /b 0

REM ============================================================
REM :DO_TRIGGER  -  ssh to board and rostopic pub smoke, y/N confirm first
REM   armed gate is enforced by board-side guidance_control (yaml: manual_smoke_bypass_armed_gate)
REM ============================================================
:DO_TRIGGER
where ssh >nul 2>&1 || (echo ERROR: ssh.exe not found & pause & exit /b 1)

echo.
echo ============================================================
echo TRIGGER SMOKE on %SSH_USER%@%TGT_IP%
echo   rostopic pub /yx/cmd/trigger_smoke std_msgs/Bool "data: true" -1
echo   (board-side armed gate is enforced by guidance_control manual_smoke_bypass_armed_gate)
echo ============================================================
set /p "CONFIRM=Are you sure you want to trigger smoke on %TGT_IP% ? [y/N] "
if /i not "%CONFIRM%"=="y" if /i not "%CONFIRM%"=="Y" if /i not "%CONFIRM%"=="yes" if /i not "%CONFIRM%"=="YES" (
    echo [trigger] cancelled by user
    pause
    endlocal
    exit /b 0
)
echo [trigger] confirmed, sending...
REM rostopic is a python3 script that needs PYTHONPATH set by sourcing setup.bash
REM bash -c (not bash -lc) is fast and avoids login shell hang
REM distro auto-detect: /opt/ros/<distro>/setup.bash
REM ROS_HOSTNAME=localhost (not $(hostname)): 板子 /etc/hosts may have stale IP
REM (e.g. 192.168.1.152) but the actual NIC is another IP (e.g. 192.168.1.156).
REM Without localhost, publisher URI resolves to the stale IP, subscriber fails
REM to connect. localhost always goes through lo 127.0.0.1, always works for
REM board-local publisher + subscriber + roscore.
REM Default -1 (no --latch, no timeout): rostopic registers with master, publishes one
REM message, then the publisher self-exits after 3s. Whole SSH command returns
REM in ~5s. No SIGTERM risk killing the publisher before XMLRPC register finishes.
ssh %SSH_OPTS% %SSH_USER%@%TGT_IP% "bash -c 'SETUP=\$(ls /opt/ros/*/setup.bash 2>/dev/null | head -1); if [ -z \"\$SETUP\" ]; then echo no /opt/ros/*/setup.bash found; exit 1; fi; source \"\$SETUP\"; export ROS_HOSTNAME=localhost; export ROS_MASTER_URI=http://localhost:11311; rostopic pub -1 /yx/cmd/trigger_smoke std_msgs/Bool \"data: true\" 2>&1'" || (
    echo ERROR: trigger failed, board-side rostopic pub reported an error (see above)
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

REM Deploy段: use the same SSH_OPTS as the rest of the script (has ControlMaster).
REM   The previous dedicated SSH_OPTS_DEPLOY had no ControlMaster, so every ssh/scp
REM   in the deploy segment re-prompted for password (10+ times per deploy).
REM   With shared master + probe at startup, deploy asks once and reuses.

echo.
echo ============================================================
echo Deploying to %TGT_IP% (SSH master reused, no password re-prompt)
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
