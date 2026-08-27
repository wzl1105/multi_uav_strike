@echo off
REM ============================================================================
REM deploy_windows.bat - 把当前目录的 tzs_ws_bundle_*.tar.gz 覆盖到目标板
REM
REM 用法:
REM   双击 deploy_windows.bat                 (会弹窗问 IP)
REM   deploy_windows.bat 192.168.1.152        (命令行直接传 IP)
REM
REM 仅用 Windows 自带工具，零依赖:
REM   tar.exe   Windows 10 1803+ 自带
REM   ssh.exe   OpenSSH Client（Win11 默认启用，Win10 1809+ 可选功能里加）
REM   scp.exe   同上
REM
REM 不依赖 WSL / Git Bash / Cygwin / 任何第三方。
REM
REM 部署前:
REM   - 把 tzs_ws_bundle_*.tar.gz 放到本脚本同一目录
REM
REM 若 ssh/scp 报错找不到:
REM   Settings ^> Apps ^> Optional Features ^> Add ^> OpenSSH Client
REM ============================================================================

setlocal enabledelayedexpansion

REM ---- 切到脚本所在目录 (双击时 current dir 不一定是这里)----
cd /d "%~dp0"

REM ---- 参数检查: 没传 IP 就弹窗问 ----
set "TGT_IP=%~1"
if "%~1"=="" (
    echo.
    echo ============================================================
    echo   tzs_ws 软更新 deploy (Windows)
    echo ============================================================
    echo   请输入目标板 IP (例如 192.168.1.152):
    echo ============================================================
    echo.
    set /p "TGT_IP=Target IP: "
    if "!TGT_IP!"=="" (
        echo ERROR: 没输入 IP, 退出
        pause
        exit /b 1
    )
)

set "SSH_USER=root"
set "TGT_BASE=/home/rpdzkj/tzs_ws"
set "SERVICES=tzs_ws-multi tzs_ws-payload"

echo.
echo [deploy] 目标板 IP: !TGT_IP!
echo.

REM ---- 检查必备工具（必须为 Windows 自带）----
where tar >nul 2>&1
if errorlevel 1 (
    echo ERROR: tar.exe not found.  Need Windows 10 1803+ or later.
    exit /b 1
)
where ssh >nul 2>&1
if errorlevel 1 (
    echo ERROR: ssh.exe not found.
    echo Install: Settings ^> Apps ^> Optional Features ^> Add ^> OpenSSH Client
    exit /b 1
)
where scp >nul 2>&1
if errorlevel 1 (
    echo ERROR: scp.exe not found.
    echo Install: Settings ^> Apps ^> Optional Features ^> Add ^> OpenSSH Client
    exit /b 1
)

REM ---- 选最新 bundle ----
set "BUNDLE="
for /f "delims=" %%F in ('dir /b /o-d tzs_ws_bundle_*.tar.gz 2^>nul') do (
    if "!BUNDLE!"=="" set "BUNDLE=%%F"
)
if "!BUNDLE!"=="" (
    echo ERROR: 当前目录没有 tzs_ws_bundle_*.tar.gz
    exit /b 1
)
echo [deploy] using bundle: !BUNDLE!

REM ---- 解压到临时目录 ----
set "WORKDIR=%TEMP%\tzs_ws_deploy_%RANDOM%"
mkdir "!WORKDIR!" 2>nul
tar -xzf "!BUNDLE!" -C "!WORKDIR!" || (
    echo ERROR: tar 解压失败
    rmdir /S /Q "!WORKDIR!" 2>nul
    exit /b 1
)
set "BUNDLE_ROOT="
for /d %%D in ("!WORKDIR!\*") do (
    if "!BUNDLE_ROOT!"=="" set "BUNDLE_ROOT=%%D"
)
if "!BUNDLE_ROOT!"=="" (
    echo ERROR: bundle 找不到根目录
    rmdir /S /Q "!WORKDIR!" 2>nul
    exit /b 1
)
echo [deploy] bundle root: !BUNDLE_ROOT!

REM ---- SSH ControlMaster: 第一次 ssh 输密码,后续 ssh/scp 都复用 ----
REM OpenSSH Windows 用 %TEMP%\cm-%r@%h-%p 作为 ControlPath
set "SSH_MASTER=%TEMP%\tzs_ws_ssh_master_%RANDOM%"
echo [deploy] opening SSH master ^(once, prompts password^)...
ssh -o ControlMaster=auto -o ControlPath=!SSH_MASTER! -o ControlPersist=600 !SSH_USER!@!TGT_IP! "true"
if errorlevel 1 (
    echo ERROR: SSH master failed
    rmdir /S /Q "!WORKDIR!" 2>nul
    exit /b 1
)
echo [deploy] SSH master ready, subsequent ssh/scp won't ask password
set "SSH_BASE=-o ControlPath=!SSH_MASTER!"

REM ---- 停服务 ----
echo [deploy] stopping services on !TGT_IP! ...
ssh !SSH_BASE! !SSH_USER!@!TGT_IP! "sudo systemctl stop !SERVICES! || true"

REM ---- 按 bundle 里实际有的包逐个 scp，不动 target 上其他包 ----
REM 重要: 不能 rm -rf devel/lib/*, 否则会把 target 上其他包也删了

REM C++ 节点: 遍历 devel/lib/<pkg>/
for /d %%P in ("!BUNDLE_ROOT!\devel\lib\*") do (
    if /i not "%%~nxP"=="python3" (
        echo [deploy] scp devel/lib\%%~nxP ...
        scp !SSH_BASE! -r "%%P\*" !SSH_USER!@!TGT_IP!:!TGT_BASE!/devel/lib/%%~nxP/
    )
)

REM Python 节点: 遍历 devel/lib/python3/dist-packages/<pkg>/
if exist "!BUNDLE_ROOT!\devel\lib\python3\dist-packages" (
    for /d %%Q in ("!BUNDLE_ROOT!\devel\lib\python3\dist-packages\*") do (
        echo [deploy] scp devel/lib\python3\dist-packages\%%~nxQ ...
        scp !SSH_BASE! -r "%%Q\*" !SSH_USER!@!TGT_IP!:!TGT_BASE!/devel/lib/python3/dist-packages/%%~nxQ/
    )
)

REM ---- src/ 如有（WITH_SRC=1 时才有，按 src/<pkg>/ 逐个 scp）----
if exist "!BUNDLE_ROOT!\src" (
    for /d %%S in ("!BUNDLE_ROOT!\src\*") do (
        echo [deploy] scp src\%%~nxS ...
        scp !SSH_BASE! -r "%%S\*" !SSH_USER!@!TGT_IP!:!TGT_BASE!/src/%%~nxS/
    )
)

REM ---- 顶层启动脚本（systemd 启动用的，必须 +x）----
if exist "!BUNDLE_ROOT!\start_gimbal.sh" (
    echo [deploy] scp + chmod +x start_gimbal.sh ...
    scp !SSH_BASE! "!BUNDLE_ROOT!\start_gimbal.sh" !SSH_USER!@!TGT_IP!:!TGT_BASE!/start_gimbal.sh
    REM 显式 chmod 兜底：Windows scp 经常丢 Linux 的执行位
    ssh !SSH_BASE! !SSH_USER!@!TGT_IP! "chmod +x !TGT_BASE!/start_gimbal.sh"
)

REM ---- 重启服务 ----
echo [deploy] restarting services on !TGT_IP! ...
ssh !SSH_BASE! !SSH_USER!@!TGT_IP! "sudo systemctl daemon-reload ^&^& sudo systemctl restart !SERVICES!"
if errorlevel 1 (
    echo WARNING: restart 返回非零，请到板子上手动 journalctl 查看
)

REM ---- 关 SSH master ----
ssh !SSH_BASE! -O exit !SSH_USER!@!TGT_IP! 2>nul

REM ---- 清理 ----
rmdir /S /Q "!WORKDIR!" 2>nul
del "!SSH_MASTER!" 2>nul

echo.
echo === DONE ===
echo 查日志:  ssh !SSH_USER!@!TGT_IP! journalctl -u tzs_ws-multi -n 30
echo 查节点:  ssh !SSH_USER!@!TGT_IP! rosnode list
echo.
echo 按任意键关闭...
pause >nul

endlocal