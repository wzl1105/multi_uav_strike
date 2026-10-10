#!/usr/bin/env bash
# LN_config.sh — 板子运维入口（Linux / WSL 双击或命令行）
#
# 菜单:
#   [D]eploy   覆盖 bundle 到板子, restart 服务
#   [Q]uery    只查板子 / 本地 package.xml 版本
#   [G]rab     停板子 rosbag record (rosnode kill), 拉最新 group 的 .bag(.active) 回本地
#   [T]rigger  手动触发拉烟 (ssh 到板子发 rostopic pub /yx/cmd/trigger_smoke), 发前 y/N 确认
#
# 用法:
#   ./LN_config.sh                       # 问 IP → 菜单
#   ./LN_config.sh <target_ip>            # IP 预填 → 菜单
#   ./LN_config.sh <target_ip> <bundle.tar.gz>  # 直 deploy(无菜单, 保留旧调用方式)
#
# 可调环境变量:
#   SSH_USER=rpdzkj               SSH 用户名（默认 rpdzkj）
#   TGT_BASE=/home/rpdzkj/tzs_ws  目标板 workspace 根（默认）
#   BAG_DIR=/home/rpdzkj/tzs_ws/LOG  板子端 bag 目录（默认, TZS_ROSBAG_BAG_DIR 风格）
#   NO_RESTART=0                  设为 1 不停服 / 不重启（仅覆盖文件，手动启停）
#   SERVICES="tzs_ws-multi tzs_ws-payload"   要 stop/restart 的列表
#
# 前置:
#   - 调试员机器能 SSH 到板子
#   - 当前目录有 tzs_ws_bundle_*.tar.gz (deploy 需要, 其他菜单不需要)
#   - SSH 用户能 sudo (deploy stop/restart 需要, 其他菜单不需要)

set -euo pipefail

SSH_USER="${SSH_USER:-root}"
TGT_BASE="${TGT_BASE:-/home/rpdzkj/tzs_ws}"
BAG_DIR="${BAG_DIR:-/home/rpdzkj/tzs_ws/LOG}"
PKG_NAME="multi_uav_strike"
PKG_XML="$TGT_BASE/src/$PKG_NAME/package.xml"
DEVEL_XML="$TGT_BASE/devel/share/$PKG_NAME/package.xml"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NO_RESTART="${NO_RESTART:-0}"
SERVICES="${SERVICES:-tzs_ws-multi tzs_ws-payload}"

# === 入口: 0 arg / 1 arg = 交互菜单, 2 arg = 旧直 deploy ===
#   ./LN_config.sh                       # 问 IP → 菜单 D/Q/G/T
#   ./LN_config.sh <ip>                  # IP 预填 → 菜单
#   ./LN_config.sh <ip> <bundle.tar.gz>  # 直 deploy (跳过菜单, 保留旧调用)
if [ "$#" -lt 2 ]; then
    if [ "$#" -lt 1 ]; then
        read -r -p "Target board IP [192.168.1.151]: " TGT_IP_INPUT
        TGT_IP="${TGT_IP_INPUT:-192.168.1.151}"
    else
        TGT_IP="$1"
    fi
    echo
    echo "What do you want to do on $TGT_IP ?"
    read -r -p "  [D]eploy (default)  /  [Q]uery version  /  [G]rab latest bag  /  [T]rigger smoke: " ACTION_CHOICE
    case "$ACTION_CHOICE" in
        [Qq]) ACTION="query" ;;
        [Gg]) ACTION="grab" ;;
        [Tt]) ACTION="trigger" ;;
        *)
            ACTION="deploy"
            BUNDLE=""
            ;;
    esac
else
    TGT_IP="$1"
    BUNDLE="$2"
    ACTION="deploy"
fi

# === SSH ControlMaster: 第一次 ssh 输密码, 后续 ssh/scp 都复用, 不再问 ===
#   query / grab / trigger / deploy 四段共用一个 master (避免每次 ssh 重新认证多问一次密码)
#   必须在所有段之前定义, query 段 (L83) 也用 $SSH_BASE
SSH_MASTER_PATH="/tmp/ssh-master-lnconfig-$$-${TGT_IP//./_}"
cleanup_ssh_master() {
    if [ -n "$SSH_MASTER_PATH" ]; then
        ssh -o ControlPath="$SSH_MASTER_PATH" -O exit "$SSH_USER@$TGT_IP" 2>/dev/null || true
        rm -f "$SSH_MASTER_PATH"
    fi
}
trap cleanup_ssh_master EXIT
SSH_BASE="ssh -o ControlMaster=auto -o ControlPath=$SSH_MASTER_PATH -o ControlPersist=600 -o ConnectTimeout=5 -o ServerAliveInterval=10 -o ServerAliveCountMax=3 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"

# ===========================================================
# Query
# ===========================================================
if [ "$ACTION" = "query" ]; then
    echo "============================================================"
    echo "Querying $PKG_NAME version on $SSH_USER@$TGT_IP"
    echo "============================================================"

    if ! ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no \
        -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR \
        "$SSH_USER@$TGT_IP" "true" 2>/dev/null; then
        echo "ERROR: cannot SSH to $SSH_USER@$TGT_IP"
        exit 1
    fi

    REMOTE_VER=""
    REMOTE_VER_FOUND=0
    # SSH only cats the file, parse <version> locally to avoid busybox grep
    # (no -P Perl regex) and avoid sed/awk escape hell over SSH.
    REMOTE_VER_RAW=$($SSH_BASE "$SSH_USER@$TGT_IP" \
        "cat '$PKG_XML' 2>/dev/null || cat '$DEVEL_XML' 2>/dev/null" 2>/dev/null || true)
    if [ -n "$REMOTE_VER_RAW" ]; then
        REMOTE_VER=$(echo "$REMOTE_VER_RAW" | grep -oP '(?<=<version>)[^<]+' | head -1 || true)
        if [ -n "$REMOTE_VER" ]; then
            REMOTE_VER_FOUND=1
        fi
        # else: file exists but has no <version> tag → REMOTE_VER stays empty
        #       and falls through to "version tag missing" branch in print
    else
        if ! $SSH_BASE "$SSH_USER@$TGT_IP" \
            "test -f '$PKG_XML' -o -f '$DEVEL_XML'" 2>/dev/null; then
            REMOTE_VER="NOT_FOUND"
        fi
    fi

    echo
    if [ "$REMOTE_VER_FOUND" = "1" ]; then
        echo "Board version  : $REMOTE_VER"
    elif [ "$REMOTE_VER" = "NOT_FOUND" ]; then
        echo "Board version  : NOT FOUND"
        echo "                 checked: $PKG_XML"
        echo "                          $DEVEL_XML"
    else
        echo "Board version  : NOT FOUND <version> tag missing in package.xml"
    fi

    echo
    echo "Inspection path : $SSH_USER@$TGT_IP:$PKG_XML"
    echo "============================================================"
    exit 0
fi

# ===========================================================
# Grab latest bag (rosnode kill + scp 回本地)
# ===========================================================
# 板子 bag 命名: multi_uav_flight_<N>_<M>.bag (.active 表示正在被写)
# 按 N 取最大那一组, 整组 (含 _active 收尾后 rename 出的 .bag + split 子文件) 全拉。
# 严禁删板子 bag, 跟 feedback_dont_delete_log_files.md 一致。
if [ "$ACTION" = "grab" ]; then
    echo "============================================================"
    echo "Grabbing latest bag from $SSH_USER@$TGT_IP:$BAG_DIR"
    echo "============================================================"
fi

# 探活 + 建 master (grab / trigger / deploy 三段都跑, query 段在前面已经用 SSH_BASE 跑过)
#   加 deploy 是因为 deploy 段之前没探活, SSH master 在第一次 deploy SSH 调用时才建,
#   多问一次密码。加 deploy 后, 启动时探活先建 master, 后续 deploy ssh/rsync 全部复用, 不再问密码
if [ "$ACTION" = "grab" ] || [ "$ACTION" = "trigger" ] || [ "$ACTION" = "deploy" ]; then
    if ! $SSH_BASE "$SSH_USER@$TGT_IP" "true" 2>/dev/null; then
        echo "ERROR: cannot SSH to $SSH_USER@$TGT_IP"
        exit 1
    fi
fi

if [ "$ACTION" = "grab" ]; then

    # 1) 找 rosbag record 进程, 拿 PID 温柔 kill (发 SIGINT, .active 收尾变 .bag)
    #   不依赖 rosnode / bash -lc (板子 ROS env 缺失 / login shell hang 都不会让 G 卡住)
    #   rosbag 进程命令行特征: "rosbag record ..." (start_gimbal.sh L754)
    echo "[grab] probing rosbag record process on board..."
    ROSBAG_PID=$($SSH_BASE "$SSH_USER@$TGT_IP" \
        "pgrep -f 'rosbag record' | head -1" 2>/dev/null | tr -d '[:space:]' || true)
    if [ -n "$ROSBAG_PID" ]; then
        echo "[grab] detected rosbag record PID: $ROSBAG_PID"
        echo "[grab] sending SIGINT (graceful, lets .active finalize to .bag)..."
        $SSH_BASE "$SSH_USER@$TGT_IP" \
            "kill -INT $ROSBAG_PID 2>&1; sleep 4; kill -0 $ROSBAG_PID 2>/dev/null && kill -KILL $ROSBAG_PID 2>/dev/null; true"
    else
        echo "[grab] no rosbag record process running, skipping kill"
    fi

    # 2) 拉一组 (按 N 最大的 group, 整组所有 .bag / .bag.active 一次 scp)
    REMOTE_BAGS=$($SSH_BASE "$SSH_USER@$TGT_IP" \
        "ls -1 '$BAG_DIR'/multi_uav_flight_*.bag* 2>/dev/null" 2>/dev/null || true)
    if [ -z "$REMOTE_BAGS" ]; then
        echo "ERROR: $BAG_DIR 下找不到 multi_uav_flight_*.bag*"
        exit 1
    fi

    # 板子端 sort -V 不一定可用 (busybox), 用 awk 切第 4 段 N 取最大组
    #   文件名 split by "_" : [multi, uav, flight, <N>, <M>.bag[.active]]
    MAX_N=$(echo "$REMOTE_BAGS" | awk -F'/' '{
        n = split($NF, a, "_")
        if (n >= 4) {
            seq_n = a[4] + 0
            if (seq_n > max_n) max_n = seq_n
        }
    } END { print max_n + 0 }')
    if [ "$MAX_N" = "0" ]; then
        echo "ERROR: 解析 bag 文件名失败, 样本:"
        echo "$REMOTE_BAGS" | head -5
        exit 1
    fi

    GROUP_FILES=$(echo "$REMOTE_BAGS" | awk -F'/' -v n="$MAX_N" '{
        parts = split($NF, a, "_")
        if (parts >= 4 && (a[4] + 0) == n) print $0
    }')
    GROUP_COUNT=$(echo "$GROUP_FILES" | grep -c '.' || true)
    echo "[grab] latest group N=$MAX_N, $GROUP_COUNT files:"
    echo "$GROUP_FILES" | sed 's/^/    /'

    # 3) scp 拉回脚本运行目录 (脚本启动时 cwd, 等价于 deploy 阶段 cd 到 SCRIPT_DIR)
    LOCAL_DEST="${BAG_GRAB_DEST:-$PWD}"
    mkdir -p "$LOCAL_DEST"
    echo "[grab] scp → $LOCAL_DEST"
    while IFS= read -r remote_file; do
        [ -n "$remote_file" ] || continue
        scp -o ControlMaster=auto -o ControlPath="$SSH_MASTER_PATH" -o ControlPersist=600 \
            -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR \
            "$SSH_USER@$TGT_IP:$remote_file" "$LOCAL_DEST/" || {
            echo "ERROR: scp 失败 $remote_file"
            exit 1
        }
    done <<< "$GROUP_FILES"

    echo
    echo "=== GRAB DONE ==="
    echo "本地目录: $LOCAL_DEST"
    ls -la "$LOCAL_DEST"/multi_uav_flight_${MAX_N}_* 2>/dev/null | sed 's/^/  /' || true
    exit 0
fi

# ===========================================================
# Trigger smoke (ssh 到板子发 rostopic pub, 发前 y/N 确认)
# ===========================================================
# armed gate 由板子端 guidance_control 自行决定 (yaml: manual_smoke_bypass_armed_gate),
# 脚本只管"按用户确认的指令转发" —— 严禁脚本绕开板子端 safety 逻辑。
if [ "$ACTION" = "trigger" ]; then
    echo "============================================================"
    echo "TRIGGER SMOKE on $SSH_USER@$TGT_IP"
    echo "  rostopic pub /yx/cmd/trigger_smoke std_msgs/Bool \"data: true\" -1"
    echo "  (板子端 armed gate 由 guidance_control manual_smoke_bypass_armed_gate 决定)"
    echo "============================================================"
    read -r -p "Are you sure you want to trigger smoke on $TGT_IP ? [y/N] " CONFIRM
    case "$CONFIRM" in
        [Yy]|[Yy][Ee][Ss])
            echo "[trigger] confirmed, sending..."
            # 关键: rostopic 是 python3 脚本, 调它必须 PYTHONPATH 包含 /opt/ros/<distro>/lib/python3/dist-packages
            # 唯一可靠的来源是 source /opt/ros/<distro>/setup.bash。
            # 之前用 'sh -c' + 直接绝对路径: sh 是 dash, source 不认, 而且跳过 source 后 PYTHONPATH 没设
            # 改用 'bash -c' (不依赖 login shell, 比 'bash -lc' 快很多, 不 hang), 显式 source distro setup.bash
            # distro 探测: ls /opt/ros/*/setup.bash 取第一个 (noetic/melodic/...)
            # ROS_HOSTNAME=localhost (不写 $(hostname)): 板子 /etc/hosts 可能写死旧 IP (e.g. 192.168.1.152)
            # 但板子网卡实际是另一个 IP (e.g. 192.168.1.156), publisher URI 解析到不存在的 IP
            # → subscriber 主动连 publisher 失败, 收不到消息。localhost 永远走 lo 127.0.0.1,
            # 同一台板子上 publisher + subscriber + roscore 互连永远通。
            # 不加 --latch, 不加 timeout: -1 模式 rostopic 会自动注册 + 发 1 条 + 3s 后自我退出
            # 之前 timeout 5 杀在 master XMLRPC 注册前 (Publishers: None); timeout 15 仍不稳
            # 改用默认 -1 行为, 整条命令 5s 内自然返回, 不靠 SIGTERM 杀
            TRIGGER_OUTPUT=$($SSH_BASE "$SSH_USER@$TGT_IP" \
                "bash -c 'SETUP=\$(ls /opt/ros/*/setup.bash 2>/dev/null | head -1); if [ -z \"\$SETUP\" ]; then echo \"no /opt/ros/*/setup.bash found\"; exit 1; fi; source \"\$SETUP\"; export ROS_HOSTNAME=localhost; export ROS_MASTER_URI=http://localhost:11311; rostopic pub -1 /yx/cmd/trigger_smoke std_msgs/Bool \"data: true\" 2>&1'" 2>&1 || true)
            echo "[trigger] board output:"
            echo "$TRIGGER_OUTPUT" | sed 's/^/    /'
            # -1 模式: publisher 注册到 master + 发 1 条 + 3s 后自退
            # "publishing and latching message for 3.0 seconds" 出现即代表已注册并开始 publish
            # 不再 info 验证 (publisher 已退, Publishers 必为 None, WARNING 是恒定误报)
            echo "=== TRIGGER DONE (板子端 subscriber 已在 latch 窗口内收到 1 条消息) ==="
            ;;
        *)
            echo "[trigger] cancelled by user"
            ;;
    esac
    exit 0
fi

# ===========================================================
# Deploy (默认)
# ===========================================================

# 选 bundle
if [ -z "$BUNDLE" ]; then
    BUNDLE=$(ls -1t tzs_ws_bundle_*.tar.gz 2>/dev/null | head -1 || true)
    if [ -z "$BUNDLE" ]; then
        echo "ERROR: 当前目录找不到 tzs_ws_bundle_*.tar.gz"
        echo "        请用 $0 <ip> <bundle.tar.gz> 显式指定"
        exit 1
    fi
    echo "[deploy] 自动选用最新 bundle: $BUNDLE"
fi

if [ ! -f "$BUNDLE" ]; then
    echo "ERROR: bundle 文件不存在: $BUNDLE"
    exit 1
fi

# SSH ControlMaster: 复用外层 grab/trigger 段的 master (L143), 避免再问一次密码
#   外层 trap cleanup_ssh_master EXIT 仍负责最后 cleanup
RSYNC_SSH="$SSH_BASE"

# 探活 (master 已建, 跑一次 true 确认还活着)
echo "[deploy] probe $SSH_USER@$TGT_IP"
if ! $SSH_BASE "$SSH_USER@$TGT_IP" "true" 2>&1; then
    echo "ERROR: 无法 SSH 到 $SSH_USER@$TGT_IP"
    exit 1
fi

# 解压到临时目录
WORK=$(mktemp -d)
# 链式 trap: deploy 退出时同时 cleanup 外层 lnconfig master + 删除临时目录
trap 'cleanup_ssh_master; rm -rf "$WORK"' EXIT
echo "[deploy] 解压 bundle → $WORK"
tar -xzf "$BUNDLE" -C "$WORK"
BUNDLE_ROOT=$(find "$WORK" -mindepth 1 -maxdepth 1 -type d | head -1)
if [ -z "$BUNDLE_ROOT" ]; then
    echo "ERROR: bundle 解压后找不到根目录（tar 包可能坏了）"
    exit 1
fi
echo "[deploy] bundle root: $BUNDLE_ROOT"

# 停服务
if [ "$NO_RESTART" = "0" ]; then
    echo "[deploy] stop: $SERVICES"
    ssh -o ControlPath="$SSH_MASTER_PATH" "$SSH_USER@$TGT_IP" "sudo systemctl stop $SERVICES || true"
fi

# rsync 覆盖 devel/
# 按 bundle 里实际有的包逐个同步，不动 target 上其他包
# 重要: 不用 --delete, 否则会把 target 上 bundle 没有的包删掉
if [ -d "$BUNDLE_ROOT/devel/lib" ]; then
    for pkg_dir in "$BUNDLE_ROOT/devel/lib"/*/; do
        [ -d "$pkg_dir" ] || continue
        pkg=$(basename "$pkg_dir")
        [ "$pkg" = "python3" ] && continue
        echo "[deploy] rsync devel/lib/$pkg/"
        rsync -av --exclude='__pycache__/' -e "$RSYNC_SSH" "$pkg_dir/" "$SSH_USER@$TGT_IP:$TGT_BASE/devel/lib/$pkg/"
    done

    # Python 节点
    if [ -d "$BUNDLE_ROOT/devel/lib/python3/dist-packages" ]; then
        for py_pkg_dir in "$BUNDLE_ROOT/devel/lib/python3/dist-packages"/*/; do
            [ -d "$py_pkg_dir" ] || continue
            py_pkg=$(basename "$py_pkg_dir")
            echo "[deploy] rsync devel/lib/python3/dist-packages/$py_pkg/"
            rsync -av --exclude='__pycache__/' -e "$RSYNC_SSH" "$py_pkg_dir/" "$SSH_USER@$TGT_IP:$TGT_BASE/devel/lib/python3/dist-packages/$py_pkg/"
        done
    fi
else
    echo "ERROR: bundle 里没有 devel/lib/ 目录，结构不对"
    exit 1
fi

# src/ 如有（WITH_SRC=1 时才会出现，按 src/<pkg>/ 逐个同步）
if [ -d "$BUNDLE_ROOT/src" ]; then
    for src_pkg_dir in "$BUNDLE_ROOT/src"/*/; do
        [ -d "$src_pkg_dir" ] || continue
        spkg=$(basename "$src_pkg_dir")
        echo "[deploy] rsync src/$spkg/"
        rsync -av \
            --exclude='__pycache__/' \
            --exclude='build/' \
            --exclude='.catkin_tools/' \
            -e "$RSYNC_SSH" \
            "$src_pkg_dir/" "$SSH_USER@$TGT_IP:$TGT_BASE/src/$spkg/"
    done
fi

# 修目录权限：rsync/scp 用 root 走完后，新建的目录常被保留为 drwx------，
# systemd 以 User=rpdzkj 跑时会进不去，roslaunch 找不到 launch/ 和 config/。
# 解法：把 src/ 和 devel/lib/ 下所有目录强制设成 755（rpdzkj 可进入）。
echo "[deploy] fix perms: chmod 755 dirs under src/ and devel/lib/"
ssh -o ControlPath="$SSH_MASTER_PATH" "$SSH_USER@$TGT_IP" \
    "find $TGT_BASE/src $TGT_BASE/devel/lib -type d -exec chmod 755 {} \;"

# 顶层启动脚本（systemd 启动用的，必须 +x）
for ts in start_gimbal.sh; do
    if [ -f "$BUNDLE_ROOT/$ts" ]; then
        echo "[deploy] copy + chmod +x  $ts"
        rsync -av -e "$RSYNC_SSH" "$BUNDLE_ROOT/$ts" "$SSH_USER@$TGT_IP:$TGT_BASE/$ts"
        # 显式 chmod 兜底：scp/rsync 偶尔会丢执行位
        ssh -o ControlPath="$SSH_MASTER_PATH" "$SSH_USER@$TGT_IP" "chmod +x $TGT_BASE/$ts"
    fi
done

# 重启服务
if [ "$NO_RESTART" = "0" ]; then
    echo "[deploy] restart: $SERVICES"
    ssh -o ControlPath="$SSH_MASTER_PATH" "$SSH_USER@$TGT_IP" "sudo systemctl daemon-reload && sudo systemctl restart $SERVICES"
fi

echo
echo "=== DONE ==="
echo "查日志:  ssh $SSH_USER@$TGT_IP journalctl -u tzs_ws-multi -n 30"
echo "查节点:  ssh $SSH_USER@$TGT_IP rosnode list"
