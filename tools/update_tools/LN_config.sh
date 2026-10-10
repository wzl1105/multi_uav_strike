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
LOCAL_XML="$SCRIPT_DIR/../../package.xml"
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
    REMOTE_VER=$(ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o LogLevel=ERROR "$SSH_USER@$TGT_IP" \
        "grep -oP '(?<=<version>)[^<]+' '$PKG_XML' 2>/dev/null \
         || grep -oP '(?<=<version>)[^<]+' '$DEVEL_XML' 2>/dev/null" 2>/dev/null | head -1 || true)
    if [ -n "$REMOTE_VER" ]; then
        REMOTE_VER_FOUND=1
    else
        if ! ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
            -o LogLevel=ERROR "$SSH_USER@$TGT_IP" \
            "test -f '$PKG_XML' -o -f '$DEVEL_XML'" 2>/dev/null; then
            REMOTE_VER="NOT_FOUND"
        fi
    fi

    LOCAL_VER=""
    if [ -f "$LOCAL_XML" ]; then
        LOCAL_VER=$(grep -oP '(?<=<version>)[^<]+' "$LOCAL_XML" 2>/dev/null | head -1 || true)
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

    if [ -n "$LOCAL_VER" ]; then
        echo "Local dev ver  : $LOCAL_VER"
    else
        echo "Local dev ver  : <not found at $LOCAL_XML>"
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

    if ! ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no \
        -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR \
        "$SSH_USER@$TGT_IP" "true" 2>/dev/null; then
        echo "ERROR: cannot SSH to $SSH_USER@$TGT_IP"
        exit 1
    fi

    # 1) 找 /record_<N> 节点, 如有就 rosnode kill (rosbag 收到 SIGINT finalize .active → .bag)
    RECORD_NODE=$(ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o LogLevel=ERROR "$SSH_USER@$TGT_IP" \
        "rosnode list 2>/dev/null | grep '^/record_' | head -1" 2>/dev/null || true)
    if [ -n "$RECORD_NODE" ]; then
        echo "[grab] killing rosbag node: $RECORD_NODE"
        ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
            -o LogLevel=ERROR "$SSH_USER@$TGT_IP" \
            "rosnode kill '$RECORD_NODE' 2>&1 || true"
        # 等 finalize
        sleep 3
    else
        echo "[grab] no /record_* node running, skipping kill"
    fi

    # 2) 拉一组 (按 N 最大的 group, 整组所有 .bag / .bag.active 一次 scp)
    REMOTE_BAGS=$(ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
        -o LogLevel=ERROR "$SSH_USER@$TGT_IP" \
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
        scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
            -o LogLevel=ERROR \
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
            ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
                -o LogLevel=ERROR "$SSH_USER@$TGT_IP" \
                "bash -lc 'source /opt/ros/*/setup.bash 2>/dev/null; rostopic pub -1 /yx/cmd/trigger_smoke std_msgs/Bool \"data: true\" 2>&1'" || {
                echo "ERROR: 触发失败, 板子端 rostopic pub 报错 (见上)"
                exit 1
            }
            echo "=== TRIGGER DONE ==="
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

# SSH ControlMaster: 第一次 ssh 输密码,后续 ssh/rsync 都复用,不再问
SSH_MASTER_PATH="/tmp/ssh-master-deploy-$$-${TGT_IP//./_}"
cleanup_ssh_master() {
    if [ -n "$SSH_MASTER_PATH" ]; then
        ssh -o ControlPath="$SSH_MASTER_PATH" -O exit "$SSH_USER@$TGT_IP" 2>/dev/null || true
        rm -f "$SSH_MASTER_PATH"
    fi
}
trap cleanup_ssh_master EXIT

# 探活 + 建 master
echo "[deploy] probe $SSH_USER@$TGT_IP"
if ! ssh -o ControlMaster=auto -o ControlPath="$SSH_MASTER_PATH" -o ControlPersist=600 -o ConnectTimeout=5 \
    "$SSH_USER@$TGT_IP" "true" 2>&1; then
    echo "ERROR: 无法 SSH 到 $SSH_USER@$TGT_IP"
    exit 1
fi

RSYNC_SSH="ssh -o ControlPath=$SSH_MASTER_PATH"

# 解压到临时目录
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
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
