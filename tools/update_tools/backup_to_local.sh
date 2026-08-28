#!/usr/bin/env bash
# backup_to_local.sh — 在开发者笔记本上跑，把指定包的编译产物从板子取到本地并打包
# +++ 默认排除 docs/ 目录（避免 Windows tar 解析 bundle 里的中文文件名炸掉）
# +++ 需临时保留 docs/ 时: EXCLUDE_DOCS=0 ./backup_to_local.sh <ip>
#
# 用法:
#   ./backup_to_local.sh <source_ip>                       # 备份三个常用包(默认不带 start_gimbal.sh)
#   ./backup_to_local.sh <source_ip> <pkg1> [pkg2 ...]     # 备份指定包
#   WITH_TOP=1 ./backup_to_local.sh 192.168.1.151          # 备份3 个包 + start_gimbal.sh
#   TOP_ONLY=1 ./backup_to_local.sh 192.168.1.151          # 只备份 start_gimbal.sh,不带 ROS 包
#
# 默认行为:
#   - 不传包名 → 默认备份 multi_uav_strike starling_bridge starling_gimbal
#   - 不带 start_gimbal.sh （改了启动脚本时显式 WITH_TOP=1）
#
# 可调环境变量:
#   SSH_USER=rpdzkj               SSH 用户名（默认 rpdzkj）
#   SRC_BASE=/home/rpdzkj/tzs_ws  板子上 workspace 根（默认）
#   WITH_SRC=0                    设为 1 同时打包 src/<pkg>/（含 launch / config 更新）
#   EXCLUDE_DOCS=1                默认 1，排除 docs/；设为 0 保留 docs/

set -euo pipefail

if [ "$#" -lt 1 ]; then
    cat <<EOF
Usage: $0 <source_ip> [pkg1 pkg2 ...]

Default behavior:
  - 不传包名 → 备份 multi_uav_strike starling_bridge starling_gimbal
  - 默认不带 start_gimbal.sh （需要时 WITH_TOP=1）
  - 默认排除 docs/ （EXCLUDE_DOCS=0 保留）

Env vars (optional):
  SSH_USER=root                 SSH user (default root)
  SRC_BASE=/home/rpdzkj/tzs_ws  Workspace root on source board
  WITH_SRC=0                    Set to 1 to also include src/<pkg>/ (launch/config)
  WITH_TOP=0                    Set to 1 to also include start_gimbal.sh
  TOP_SCRIPTS="start_gimbal.sh foo.sh"  Custom list of top-level scripts
  TOP_ONLY=0                    Set to 1 to skip packages, only backup top-level scripts
  EXCLUDE_DOCS=1                Set to 0 to include docs/ (may contain CN filenames)

Example:
  $0 192.168.1.151
  $0 192.168.1.151 multi_uav_strike
  WITH_TOP=1 $0 192.168.1.151 multi_uav_strike starling_bridge
  TOP_ONLY=1 $0 192.168.1.151
  WITH_SRC=1 $0 192.168.1.151 multi_uav_strike starling_bridge
  EXCLUDE_DOCS=0 $0 192.168.1.151       # 保留 docs/（含中文文档）
EOF
    exit 1
fi

SRC_IP="$1"
shift
PKGS=("$@")

# 不传包名 → 默认备份三个常用包
DEFAULT_PKGS=(multi_uav_strike starling_bridge starling_gimbal)
if [ "${#PKGS[@]}" -eq 0 ]; then
    PKGS=("${DEFAULT_PKGS[@]}")
    echo "[info] 未指定包名，默认备份: ${PKGS[*]}"
    echo "[info] 显式指定: $0 <ip> <pkg1> [pkg2 ...]"
    echo
fi

# TOP_ONLY=1 → 不备份任何 ROS 包，只备份顶层脚本（如 start_gimbal.sh）
if [ "${TOP_ONLY:-0}" = "1" ]; then
    PKGS=()
    echo "[info] TOP_ONLY=1: 仅备份顶层脚本，不备份 ROS 包"
    echo
fi

SSH_USER="${SSH_USER:-root}"
SRC_BASE="${SRC_BASE:-/home/rpdzkj/tzs_ws}"
WITH_SRC="${WITH_SRC:-0}"
EXCLUDE_DOCS="${EXCLUDE_DOCS:-1}"           # +++

# +++ docs/ 排除项（默认启用，避免 bundle 里有中文文件名）
DOCS_EXCLUDE=()
if [ "$EXCLUDE_DOCS" = "1" ]; then
    DOCS_EXCLUDE=(--exclude='docs/')
    echo "[info] EXCLUDE_DOCS=1: 排除 docs/（避免 Windows tar 解析中文文件名炸）"
    echo
fi

# SSH ControlMaster: 第一次 ssh 输密码,后续 ssh/rsync 都复用这个连接,不再问密码
SSH_MASTER_PATH="/tmp/ssh-master-backup-$$-${SRC_IP//./_}"
cleanup_ssh_master() {
    if [ -n "$SSH_MASTER_PATH" ]; then
        ssh -o ControlPath="$SSH_MASTER_PATH" -O exit "$SSH_USER@$SRC_IP" 2>/dev/null || true
        rm -f "$SSH_MASTER_PATH"
    fi
}
trap cleanup_ssh_master EXIT
ssh -o ControlMaster=auto -o ControlPath="$SSH_MASTER_PATH" -o ControlPersist=600 \
    "$SSH_USER@$SRC_IP" "true" 2>&1 | grep -v "password:" || true
# 导出给 rsync 用
RSYNC_SSH="ssh -o ControlPath=$SSH_MASTER_PATH"

# 工作目录：当前路径下
WORK="tzs_ws_bundle_${SRC_IP}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$WORK/devel/lib" "$WORK/devel/lib/python3/dist-packages"

echo "=== backup_to_local ==="
echo "src  : $SSH_USER@$SRC_IP:$SRC_BASE"
echo "pkgs : ${PKGS[*]}"
echo "out  : ./$WORK/"
echo "WITH_SRC=$WITH_SRC  (set WITH_SRC=1 to include launch/config)"
echo "EXCLUDE_DOCS=$EXCLUDE_DOCS"
echo

# 探活 (用已建好的 ControlMaster)
if ! ssh -o ControlPath="$SSH_MASTER_PATH" -o ConnectTimeout=5 "$SSH_USER@$SRC_IP" "true" 2>/dev/null; then
    echo "ERROR: 无法 SSH 到 $SSH_USER@$SRC_IP"
    exit 1
fi

for pkg in "${PKGS[@]}"; do
    echo "--- $pkg ---"

    # C++ 节点：wrapper 脚本 + .so
    SRC="${SSH_USER}@${SRC_IP}:${SRC_BASE}/devel/lib/${pkg}/"
    DST="$WORK/devel/lib/${pkg}"
    if ssh -o ControlPath="$SSH_MASTER_PATH" "$SSH_USER@$SRC_IP" "test -d ${SRC_BASE}/devel/lib/${pkg}" 2>/dev/null; then
        mkdir -p "$DST"
        rsync -av --delete --exclude='__pycache__/' "${DOCS_EXCLUDE[@]}" \
            -e "$RSYNC_SSH" "$SRC" "$DST/"                        # +++
        echo "[OK]  devel/lib/$pkg/  (C++ wrapper + .so)"
    else
        echo "[--]  无 devel/lib/$pkg/  (纯 Python 包?跳过)"
    fi

    # Python 节点（如有）
    PY_SRC="${SSH_USER}@${SRC_IP}:${SRC_BASE}/devel/lib/python3/dist-packages/${pkg}/"
    PY_DST="$WORK/devel/lib/python3/dist-packages/${pkg}"
    if ssh -o ControlPath="$SSH_MASTER_PATH" "$SSH_USER@$SRC_IP" "test -d ${SRC_BASE}/devel/lib/python3/dist-packages/${pkg}" 2>/dev/null; then
        mkdir -p "$PY_DST"
        rsync -av --delete --exclude='__pycache__/' "${DOCS_EXCLUDE[@]}" \
            -e "$RSYNC_SSH" "$PY_SRC" "$PY_DST/"                   # +++
        echo "[OK]  devel/lib/python3/dist-packages/$pkg/  (Python)"
    else
        echo "[--]  无 Python 节点目录"
    fi

    # src/<pkg>/ 仅在 WITH_SRC=1 时拷（包含 launch / config）
    if [ "$WITH_SRC" = "1" ]; then
        SRC_PKG="${SSH_USER}@${SRC_IP}:${SRC_BASE}/src/${pkg}/"
        DST_PKG="$WORK/src/${pkg}"
        if ssh -o ControlPath="$SSH_MASTER_PATH" "$SSH_USER@$SRC_IP" "test -d ${SRC_BASE}/src/${pkg}" 2>/dev/null; then
            mkdir -p "$WORK/src"
            rsync -av --delete \
                --exclude='__pycache__/' \
                --exclude='build/' \
                --exclude='.catkin_tools/' \
                --exclude='.git/' \
                --exclude='.vscode/' \
                "${DOCS_EXCLUDE[@]}" \
                -e "$RSYNC_SSH" \
                "$SRC_PKG" "$DST_PKG/"                              # +++
            echo "[OK]  src/$pkg/  (含 launch / config)"
        else
            echo "[!!]  WITH_SRC=1 但板子上没找到 src/$pkg/"
        fi
    fi
    echo
done

# 顶层启动脚本（systemd ExecStart 指向的，必须可执行）
# 默认不拷，需要时再显式开 WITH_TOP=1
# 可通过 TOP_SCRIPTS="start_gimbal.sh foo.sh" 指定要拷的脚本列表
TOP_SCRIPTS="${TOP_SCRIPTS:-}"
if [ "${WITH_TOP:-0}" = "1" ] && [ -z "$TOP_SCRIPTS" ]; then
    TOP_SCRIPTS="start_gimbal.sh"
fi
if [ -n "$TOP_SCRIPTS" ]; then
    echo "--- top-level scripts ---"
    for ts in $TOP_SCRIPTS; do
        if ssh -o ControlPath="$SSH_MASTER_PATH" "$SSH_USER@$SRC_IP" "test -f ${SRC_BASE}/$ts" 2>/dev/null; then
            rsync -av -e "$RSYNC_SSH" "${SSH_USER}@${SRC_IP}:${SRC_BASE}/$ts" "$WORK/"
            # 保留执行位（rsync -a 已经保留了，这里只是显式确认）
            chmod +x "$WORK/$ts"
            echo "[OK]  $ts  (顶层启动脚本，已保留 +x)"
        else
            echo "[--]  板子上没找到 $ts"
        fi
    done
else
    echo "--- top-level scripts: 跳过（默认不带,需要时 WITH_TOP=1）---"
fi
echo

# 把 deploy 脚本一起打进 bundle（调试员解压就能用，不用单独发）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
for deploy_s in deploy_linux.sh deploy_windows.bat; do
    if [ -f "$SCRIPT_DIR/$deploy_s" ]; then
        cp "$SCRIPT_DIR/$deploy_s" "$WORK/"
        echo "[OK]  $deploy_s  (随 bundle 一起发)"
    else
        echo "[!!]  没找到 $SCRIPT_DIR/$deploy_s，调试员需另外获取"
    fi
done

# 生成 README.txt（替换占位符）
if [ -f "$SCRIPT_DIR/README.txt" ]; then
    PKG_LIST=$(printf "%s, " "${PKGS[@]}"); PKG_LIST="${PKG_LIST%, }"
    sed -e "s|__SRC_IP__|$SRC_IP|g" \
        -e "s|__TIMESTAMP__|$(date '+%Y-%m-%d %H:%M:%S')|g" \
        -e "s|__PKG_LIST__|$PKG_LIST|g" \
        "$SCRIPT_DIR/README.txt" > "$WORK/README.txt"
    echo "[OK]  README.txt  (使用说明)"
else
    echo "[!!]  没找到 README.txt 模板"
fi
echo

# 打包
TARBALL="${WORK}.tar.gz"
echo "=== packing → $TARBALL ==="
tar -czf "$TARBALL" -C "$(dirname "$WORK")" "$(basename "$WORK")"

echo
echo "=== DONE ==="
ls -lh "$TARBALL"
echo "  bundle 内: $WORK/"
du -sh "$WORK" 2>/dev/null || true
echo
echo "把 $TARBALL 发给调试员即可。"
echo "他们在 bundle 同目录下执行 deploy_linux.sh <ip> 或 deploy_windows.bat <ip>。"
