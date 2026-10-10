#!/bin/bash
# ============================================
# 载荷自启动 / multi_uav_strike 启动停止脚本
# 由独立的 systemd service 分别调用 start-payload / start-multi
# 兼容旧 service：start-gimbal/stop-gimbal 等价于 start-payload/stop-payload，
# 但不会启动、停止或等待 starling_gimbal
# ROS 仅本机：使用 localhost，与 eth0 IP 完全解耦
# 关键改进：
#   1. payload 和 multi 在不同 cgroup，互不影响
#   2. cleanup 改用 PID 精确杀，避免 pkill 误伤还在退出的进程
#   3. cleanup 后等真正退出（最多 20s），避免 tty 抢占 race
# 注意：不能用 set -u，ROS catkin profile.d 会触发 unbound variable
# ============================================

LOG_DIR=/var/log/tzs_ws
PAYLOAD_PIDFILE=/run/tzs_ws/payload.pid
MULTI_PIDFILE=/run/tzs_ws/multi.pid
ROSBAG_PIDFILE=/run/tzs_ws/rosbag.pid
ROSBAG_BAG_DIR="${TZS_ROSBAG_BAG_DIR:-/home/rpdzkj/tzs_ws/LOG}"
ROSBAG_BAG_PREFIX="${TZS_ROSBAG_BAG_PREFIX:-multi_uav_flight}"
ROSBAG_EXCLUDE_REGEX='.*(/ros_short_image$|/ros_ir_image$|/ros_long_image$|/ros_vis_image_res$|/ros_ir_image_res$|/ros_zoom_image$|/starling_gimbal/training_sample$).*'
ROSBAG_START_DELAY="${TZS_ROSBAG_START_DELAY:-15}"
ENV_FILE=/run/tzs_ws/env
RS485_PORT="${TZS_RS485_PORT:-/dev/ttyS3}"
RS485_GPIO="${TZS_RS485_GPIO:-50}"  # Rockchip GPIO1_C2 = 1*32 + 2*8 + 2
RS485_GPIO_CHIP="${TZS_RS485_GPIO_CHIP:-gpiochip1}"
RS485_GPIO_LINE="${TZS_RS485_GPIO_LINE:-18}"  # GPIO1_C2 在 gpiochip1 内的偏移
RS485_GPIO_PIDFILE=/run/tzs_ws/rs485_gpio.pid
RS485_GPIO_HOLDER_PID=""
RS485_BAUD="${TZS_RS485_BAUD:-115200}"
RS485_INITIAL_TIMEOUT="${TZS_RS485_INITIAL_TIMEOUT:-5}"
# 长监听 RS485 的默认超时：旧版本 300s → 30s → 10s → 2026-08-27 改回 30s。
# 没有载荷时整套 detect_payload 总耗时 ≈ 5(短监听) + ~1(IP 扫描) + 30(长监听) ≈ 36s。
# 想覆盖：export TZS_RS485_DETECT_TIMEOUT=N 之类。
RS485_DETECT_TIMEOUT="${TZS_RS485_DETECT_TIMEOUT:-30}"
DATALINK_PORT="${TZS_DATALINK_PORT:-/dev/ttyS8}"
DATALINK_BAUD="${TZS_DATALINK_BAUD:-115200}"
DATALINK_SETTLE_SEC="${TZS_DATALINK_SETTLE_SEC:-2}"
GPIO_SYSFS_ROOT="${TZS_GPIO_SYSFS_ROOT:-/sys/class/gpio}"
mkdir -p "$LOG_DIR" "$(dirname "$ENV_FILE")"
export ROS_DISTRO=noetic

# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
# shellcheck source=/dev/null
source /home/rpdzkj/tzs_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

# 映像版本号 (内部版本管理用, apply_fix.sh 部署时写入)
# 默认值只此一处, heredoc 里直接引用 $TZS_IMAGE_VERSION
: "${TZS_IMAGE_VERSION:=1.1.1}"
export TZS_IMAGE_VERSION

log() { echo "[$(date '+%F %T')] $*"; }
fail() { log "ERROR: $*"; return 1; }

write_env_file() {
    cat > "$ENV_FILE" <<EOF
# 自动生成于 $(date '+%F %T')
# 使用方法：source /run/tzs_ws/env
export TZS_ETH0_IP="${TZS_ETH0_IP:-}"
export TZS_DEVICE_SN="${TZS_DEVICE_SN:-1-1}"
export TZS_FCU_ID=${TZS_FCU_ID:-1}
export TZS_PAYLOAD_TYPE="${TZS_PAYLOAD_TYPE:-none}"
export TZS_PAYLOAD_ENDPOINT="${TZS_PAYLOAD_ENDPOINT:-}"
export TZS_RS485_PORT="$RS485_PORT"
export TZS_RS485_GPIO="$RS485_GPIO"
export TZS_RS485_GPIO_CHIP="$RS485_GPIO_CHIP"
export TZS_RS485_GPIO_LINE="$RS485_GPIO_LINE"
export TZS_RS485_BAUD="$RS485_BAUD"
export TZS_RS485_INITIAL_TIMEOUT="$RS485_INITIAL_TIMEOUT"
export TZS_RS485_DETECT_TIMEOUT="$RS485_DETECT_TIMEOUT"
export TZS_DATALINK_PORT="$DATALINK_PORT"
export TZS_DATALINK_BAUD="$DATALINK_BAUD"
export TZS_IMAGE_VERSION="$TZS_IMAGE_VERSION"
EOF
    chmod 0644 "$ENV_FILE" 2>/dev/null || true
}

# 无条件刷新一次 env 文件, 让任何入口 (start-multi / start-payload / stop-*) 都能保证
# /run/tzs_ws/env 与当前 TZS_IMAGE_VERSION 同步, 避免 SSH echo 看到旧值
write_env_file

# 根据 eth0 IP 推算：
#   TZS_DEVICE_SN = "<group>-<sub>"，每 6 个 IP 一组
#       151-156 → 1-1..1-6；157-162 → 2-1..2-6；163 → 3-1 ...
#   TZS_FCU_ID   = 线性编号，151→1，162→12
# 用户改 IP 只改 /etc/netplan/*.yaml 的 eth0 addresses（标准做法）
detect_ip_from_netplan() {
    local ip=""
    # 用 python3+PyYAML 解析所有 /etc/netplan/*.yaml，找 ethernets 下任意接口的 addresses 第一项
    local script='
import yaml, sys, glob, os
for path in sorted(glob.glob("/etc/netplan/*.yaml") + glob.glob("/etc/netplan/*.yml")):
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
    except Exception:
        continue
    if not isinstance(data, dict):
        continue
    n = data.get("network") or {}
    for ifname, ifconf in (n.get("ethernets") or {}).items():
        if not isinstance(ifconf, dict):
            continue
        for a in (ifconf.get("addresses") or []):
            if isinstance(a, str) and "/" in a and not a.startswith(("fe80", "127.")):
                print(a.split("/")[0])
                sys.exit(0)
sys.exit(1)
'
    ip=$(python3 -c "$script" 2>/dev/null | head -1)
    echo "$ip"
}

derive_tzs_env() {
    local ip=""

    # 1. /etc/netplan/*.yaml（标准做法，你改 IP 就是改这里）
    ip=$(detect_ip_from_netplan)
    [ -n "$ip" ] && log "从 /etc/netplan/*.yaml 读到 eth0 IP: $ip"

    # 2. fallback: /etc/network/interfaces（兼容老式 ifupdown 配置的板子）
    if [ -z "$ip" ] && [ -r /etc/network/interfaces ]; then
        ip=$(awk '/^[ \t]*address[ \t]+/ {print $2; exit}' /etc/network/interfaces 2>/dev/null)
        [ -n "$ip" ] && log "从 /etc/network/interfaces 读到 eth0 IP: $ip"
    fi

    # 3. fallback: 默认路由 src IP
    if [ -z "$ip" ]; then
        ip=$(ip -4 route get 8.8.8.8 2>/dev/null | awk '/src/ {for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')
        [ -n "$ip" ] && log "从默认路由 src 读到 eth0 IP: $ip"
    fi

    # 4. fallback: eth0 上第一个 global IP（非 secondary）
    if [ -z "$ip" ]; then
        ip=$(ip -4 -o addr show dev eth0 scope global 2>/dev/null | grep -v secondary | head -1 | awk '{print $4}' | cut -d/ -f1)
        [ -n "$ip" ] && log "从 eth0 接口读到 IP: $ip"
    fi

    if [ -z "$ip" ]; then
        log "WARN: 无法确定 eth0 IP，使用默认 TZS_DEVICE_SN=1-1 TZS_FCU_ID=1"
        export TZS_DEVICE_SN="1-1"
        export TZS_FCU_ID=1
        export TZS_ETH0_IP=""
        write_env_file
        return 0
    fi

    local last="${ip##*.}"
    if ! [[ "$last" =~ ^[0-9]+$ ]]; then
        log "WARN: 无法解析 eth0=$ip 的末位数字，回退默认"
        export TZS_DEVICE_SN="1-1"
        export TZS_FCU_ID=1
        export TZS_ETH0_IP="$ip"
        write_env_file
        return 0
    fi

    # 范围越界也照算公式，但打 WARN
    if [ "$last" -lt 151 ] || [ "$last" -gt 254 ]; then
        log "WARN: eth0=$ip 末位 $last 超出常规划分区间 (151-254)，仍按公式计算"
    fi

    local offset=$((last - 151))
    local group=$((offset / 6 + 1))
    local sub=$((offset % 6 + 1))
    export TZS_DEVICE_SN="${group}-${sub}"
    export TZS_FCU_ID=$((last - 150))
    export TZS_ETH0_IP="$ip"

    log "eth0=$ip → TZS_DEVICE_SN=$TZS_DEVICE_SN, TZS_FCU_ID=$TZS_FCU_ID"
    write_env_file
    return 0
}

# 并行探测两段载荷 IP。每轮最慢约为单个 ping 的 1 秒超时。
scan_payload_ips() {
    local last ip
    for last in $(seq 171 182) $(seq 221 232); do
        ip="192.168.1.$last"
        (
            if ping -n -c 1 -W 1 "$ip" >/dev/null 2>&1; then
                echo "$ip"
            fi
        ) &
    done
    wait
}

release_rs485_gpio() {
    local pid="${RS485_GPIO_HOLDER_PID:-}"
    local cmdline=""
    local _

    if [ -z "$pid" ] && [ -r "$RS485_GPIO_PIDFILE" ]; then
        pid=$(cat "$RS485_GPIO_PIDFILE" 2>/dev/null)
    fi
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
        cmdline=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
        if [[ "$cmdline" == *gpioset* ]]; then
            kill -TERM "$pid" 2>/dev/null || true
            for _ in $(seq 1 20); do
                kill -0 "$pid" 2>/dev/null || break
                sleep 0.05
            done
            kill -KILL "$pid" 2>/dev/null || true
        fi
    fi
    rm -f "$RS485_GPIO_PIDFILE"
    RS485_GPIO_HOLDER_PID=""
}

set_rs485_receive_mode() {
    local gpio_dir="$GPIO_SYSFS_ROOT/gpio$RS485_GPIO"
    local gpioset_error_file="" gpioset_error="" gpioset_rc=0
    local gpioset_child_pid=""
    local _

    if ! [[ "$RS485_GPIO_LINE" =~ ^[0-9]+$ ]]; then
        log "ERROR: TZS_RS485_GPIO_LINE=$RS485_GPIO_LINE 不是有效行偏移"
        return 1
    fi

    # 板端首选已验证的 libgpiod 接口。-m signal 会持续占有该行并保持低电平。
    if command -v gpioset >/dev/null 2>&1; then
        release_rs485_gpio
        gpioset_error_file=$(mktemp "$(dirname "$ENV_FILE")/gpioset.XXXXXX")
        gpioset -m signal -b "$RS485_GPIO_CHIP" "$RS485_GPIO_LINE=0" \
            >/dev/null 2>"$gpioset_error_file" &
        RS485_GPIO_HOLDER_PID=$!
        sleep 0.2
        if kill -0 "$RS485_GPIO_HOLDER_PID" 2>/dev/null; then
            echo "$RS485_GPIO_HOLDER_PID" > "$RS485_GPIO_PIDFILE"
            rm -f "$gpioset_error_file"
            log "GPIO1_C2 已通过 $RS485_GPIO_CHIP line $RS485_GPIO_LINE 设置为低电平"
            return 0
        fi
        if wait "$RS485_GPIO_HOLDER_PID"; then
            gpioset_rc=0
        else
            gpioset_rc=$?
        fi
        gpioset_error=$(tr '\n' ' ' < "$gpioset_error_file" 2>/dev/null)
        rm -f "$gpioset_error_file"
        RS485_GPIO_HOLDER_PID=""

        # 部分板端 gpioset 会让启动进程正常退出，并由派生进程继续持有 GPIO；
        # 也有版本在成功设置后直接返回 0。两种情况都不能回退到 sysfs。
        if [ "$gpioset_rc" -eq 0 ]; then
            gpioset_child_pid=$(pgrep -n -x gpioset 2>/dev/null || true)
            if [[ "$gpioset_child_pid" =~ ^[0-9]+$ ]] &&
               kill -0 "$gpioset_child_pid" 2>/dev/null; then
                RS485_GPIO_HOLDER_PID="$gpioset_child_pid"
                echo "$RS485_GPIO_HOLDER_PID" > "$RS485_GPIO_PIDFILE"
                log "GPIO1_C2 已通过派生 gpioset PID=$RS485_GPIO_HOLDER_PID 保持低电平"
            else
                log "GPIO1_C2 的 gpioset 命令已成功完成并返回 0"
            fi
            return 0
        fi

        log "WARN: gpioset 设置 GPIO1_C2 失败 (code=$gpioset_rc): ${gpioset_error:-<no error text>}"
        log "WARN: 尝试旧 sysfs GPIO$RS485_GPIO"
    fi

    if ! [[ "$RS485_GPIO" =~ ^[0-9]+$ ]]; then
        log "ERROR: TZS_RS485_GPIO=$RS485_GPIO 不是有效 GPIO 编号"
        return 1
    fi
    if [ ! -d "$GPIO_SYSFS_ROOT" ]; then
        log "ERROR: GPIO sysfs 不存在: $GPIO_SYSFS_ROOT"
        return 1
    fi

    if [ ! -d "$gpio_dir" ]; then
        if ! echo "$RS485_GPIO" > "$GPIO_SYSFS_ROOT/export" 2>/dev/null; then
            log "ERROR: 无法导出 GPIO$RS485_GPIO（GPIO1_C2）"
            return 1
        fi
        for _ in $(seq 1 20); do
            [ -d "$gpio_dir" ] && break
            sleep 0.05
        done
    fi

    if [ ! -w "$gpio_dir/direction" ] || [ ! -w "$gpio_dir/value" ]; then
        log "ERROR: GPIO$RS485_GPIO direction/value 不可写"
        return 1
    fi

    # direction=low 会原子地切为输出低电平；不支持 low 的旧驱动回退为 out + value=0。
    if ! echo low > "$gpio_dir/direction" 2>/dev/null; then
        echo out > "$gpio_dir/direction" 2>/dev/null || return 1
    fi
    echo 0 > "$gpio_dir/value" 2>/dev/null || return 1

    if [ "$(cat "$gpio_dir/value" 2>/dev/null)" != "0" ]; then
        log "ERROR: GPIO$RS485_GPIO 未能保持低电平"
        return 1
    fi
    log "GPIO1_C2(GPIO$RS485_GPIO) 已设置为低电平，RS485 进入接收模式"
    return 0
}

# =============================================================================
# YX 485 心跳帧验证 (Python 内联脚本)
# 上行帧格式 (YX → 机载): AA 55 | 02 | 状态字0 | 预留[11] | 累加和[1]  = 16 字节
# 校验: 字节 0~14 累加和的低 8 位 == 字节 15
# =============================================================================
YX_UL_FRAME_LEN=16
YX_UL_SYNC_HI=0xAA
YX_UL_SYNC_LO=0x55
YX_UL_ADDR=0x02

rs485_receive_data() {
    local timeout="${1:-$RS485_DETECT_TIMEOUT}"
    python3 - "$RS485_PORT" "$RS485_BAUD" "$timeout" \
              "$YX_UL_FRAME_LEN" "$YX_UL_SYNC_HI" "$YX_UL_SYNC_LO" "$YX_UL_ADDR" <<'PY'
import os
import select
import sys
import termios
import tty

fd = None
try:
    port = sys.argv[1]
    baud = int(sys.argv[2])
    timeout = float(sys.argv[3])
    frame_len = int(sys.argv[4])
    # 允许十进制或 0x 前缀十六进制
    sync_hi = int(sys.argv[5], 0)
    sync_lo = int(sys.argv[6], 0)
    addr_yx = int(sys.argv[7], 0)

    speed = getattr(termios, "B{}".format(baud), None)
    if speed is None or timeout <= 0:
        raise ValueError("unsupported baud or timeout")

    fd = os.open(port, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
    tty.setraw(fd, termios.TCSANOW)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    attrs[2] |= termios.CS8
    attrs[4] = speed
    attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)

    # 滑动窗口缓冲区，持续读取直到超时或找到合法心跳帧
    buf = bytearray()
    import time as _time
    deadline = _time.monotonic() + timeout
    while _time.monotonic() < deadline:
        remaining = deadline - _time.monotonic()
        if remaining <= 0:
            break
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            break  # timeout
        chunk = os.read(fd, 256)
        if not chunk:
            continue
        buf.extend(chunk)

        # 滑动窗口搜索合法 YX 上行心跳帧
        while len(buf) >= frame_len:
            # 找同步头 AA 55
            pos = -1
            for i in range(len(buf) - 1):
                if buf[i] == sync_hi and buf[i + 1] == sync_lo:
                    pos = i
                    break
            if pos < 0:
                # 没有同步头，只保留最后 1 字节（可能是 AA 的起始）
                buf = buf[-1:]
                break
            if pos + frame_len > len(buf):
                # 凑不齐一帧，保留从同步头开始的数据等下一波
                buf = buf[pos:]
                break

            # 提取候选帧
            frame = buf[pos:pos + frame_len]
            buf = buf[pos + frame_len:]  # 消费掉

            # 地址码检查
            if frame[2] != addr_yx:
                continue

            # 累加和校验：字节 0~14 累加，低 8 位 == 字节 15
            cksum = sum(frame[:frame_len - 1]) & 0xFF
            if cksum != frame[frame_len - 1]:
                continue

            # 合法心跳帧！
            print(frame.hex())
            sys.exit(0)

    # 超时，未收到合法心跳帧
    sys.exit(1)
except (OSError, ValueError, termios.error) as exc:
    print("RS485 serial error: {}".format(exc), file=sys.stderr)
    sys.exit(2)
finally:
    if fd is not None:
        os.close(fd)
PY
}

send_datalink_power_command() {
    local action="$1"
    local frame_hex=""
    local action_label=""

    case "$action" in
        down) frame_hex="7100020000735c"; action_label="下电" ;;
        up)   frame_hex="7100010000725c"; action_label="上电" ;;
        *)
            log "ERROR: 未知数据链电源指令: $action"
            return 1
            ;;
    esac

    if python3 - "$DATALINK_PORT" "$DATALINK_BAUD" "$frame_hex" <<'PY'
import os
import sys
import termios
import tty

fd = None
try:
    port = sys.argv[1]
    baud = int(sys.argv[2])
    frame = bytes.fromhex(sys.argv[3])
    speed = getattr(termios, "B{}".format(baud), None)
    if speed is None:
        raise ValueError("unsupported baud")

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd, termios.TCSANOW)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    attrs[2] |= termios.CS8
    attrs[4] = speed
    attrs[5] = speed
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    sent = 0
    while sent < len(frame):
        sent += os.write(fd, frame[sent:])
    termios.tcdrain(fd)
except (OSError, ValueError, termios.error) as exc:
    print("datalink serial error: {}".format(exc), file=sys.stderr)
    sys.exit(1)
finally:
    if fd is not None:
        os.close(fd)
PY
    then
        log "数据链${action_label}指令已发送到 $DATALINK_PORT: $frame_hex"
        return 0
    fi

    log "ERROR: 数据链${action_label}指令发送失败"
    return 1
}

set_payload_result() {
    export TZS_PAYLOAD_TYPE="$1"
    export TZS_PAYLOAD_ENDPOINT="$2"
    write_env_file
    log "载荷识别结果: type=$TZS_PAYLOAD_TYPE endpoint=${TZS_PAYLOAD_ENDPOINT:-<none>}"
}

detect_payload() {
    local override="${TZS_PAYLOAD_OVERRIDE:-}"
    local found="" count=0 ip="" last=0
    local conflict_endpoints=""
    local rs485_data="" rs485_rc=0
    local rs485_ready="false"

    # 清理由本脚本上一次异常退出可能遗留的 gpioset 持有进程。
    release_rs485_gpio

    if [ -n "$override" ]; then
        case "$override" in
            lidar|relay)
                log "使用 TZS_PAYLOAD_OVERRIDE=$override，跳过硬件探测"
                set_payload_result "$override" "override"
                return 0
                ;;
            rs485)
                log "使用 TZS_PAYLOAD_OVERRIDE=rs485，跳过硬件探测"
                set_payload_result "rs485" "$RS485_PORT"
                return 0
                ;;
            none)
                log "使用 TZS_PAYLOAD_OVERRIDE=none，跳过硬件探测"
                set_payload_result "none" ""
                return 0
                ;;
            *)
                log "ERROR: 无效 TZS_PAYLOAD_OVERRIDE=$override，按 none 处理"
                set_payload_result "none" ""
                return 0
                ;;
        esac
    fi

    # 阶段1：先将 RS485 置为接收方向，短暂等待载荷主动上报。
    if set_rs485_receive_mode; then
        rs485_ready="true"
        log "首次监听 RS485 端口 $RS485_PORT（${RS485_BAUD}-8N1，${RS485_INITIAL_TIMEOUT}s）"
        if rs485_data=$(rs485_receive_data "$RS485_INITIAL_TIMEOUT"); then
            log "首次监听收到 RS485 数据（${#rs485_data} 个十六进制字符）"
            set_payload_result "rs485" "$RS485_PORT"
            return 0
        else
            rs485_rc=$?
            if [ "$rs485_rc" -eq 1 ]; then
                log "首次 ${RS485_INITIAL_TIMEOUT}s 内未收到 RS485 数据"
            else
                log "首次 RS485 串口打开或配置失败 (code=$rs485_rc)"
                rs485_ready="false"
            fi
        fi
    else
        log "RS485 接收方向配置失败，继续检测 IP 载荷"
    fi

    # 阶段2：首次没有 RS485 数据，直接扫描一次两个 IP 地址池。
    # 注 (2026-08-27)：数据链上下电功能已停用，下面两行（关数据链 + 下电 settle）已禁用。
    # send_datalink_power_command down || log "WARN: 数据链下电失败，仍继续扫描 IP 地址池"
    # sleep "$DATALINK_SETTLE_SEC"

    log "开始扫描通信中继 192.168.1.171-182 和激光雷达 192.168.1.221-232"
    found=$(scan_payload_ips | sort -t. -k4,4n)
    count=$(printf '%s\n' "$found" | awk 'NF {n++} END {print n+0}')

    if [ "$count" -gt 1 ]; then
        release_rs485_gpio
        log "WARN: 检测到多个 IP 载荷，拒绝选择: $(printf '%s' "$found" | tr '\n' ' ')"
        conflict_endpoints=$(printf '%s\n' "$found" | paste -sd, -)
        set_payload_result "conflict" "$conflict_endpoints"
        return 0
    fi

    if [ "$count" -eq 1 ]; then
        release_rs485_gpio
        ip="$found"
        last="${ip##*.}"
        if [ "$last" -ge 171 ] && [ "$last" -le 182 ]; then
            set_payload_result "relay" "$ip"
        else
            set_payload_result "lidar" "$ip"
        fi
        return 0
    fi

    # 阶段3：两个 IP 池均无设备，持续监听 RS485。
    # 注 (2026-08-27)：数据链上下电功能已停用，恢复上电 + 上电 settle 已禁用。
    log "两个 IP 地址池均未检测到载荷，继续监听 RS485"
    # send_datalink_power_command up || log "WARN: 数据链上电失败，仍继续等待 RS485"
    # sleep "$DATALINK_SETTLE_SEC"

    if [ "$rs485_ready" != "true" ]; then
        if set_rs485_receive_mode; then
            rs485_ready="true"
        fi
    fi

    if [ "$rs485_ready" = "true" ]; then
        log "继续监听 RS485 端口 $RS485_PORT（最多 ${RS485_DETECT_TIMEOUT}s）"
        if rs485_data=$(rs485_receive_data "$RS485_DETECT_TIMEOUT"); then
            log "持续监听收到 RS485 数据（${#rs485_data} 个十六进制字符）"
            set_payload_result "rs485" "$RS485_PORT"
            return 0
        else
            rs485_rc=$?
            if [ "$rs485_rc" -eq 1 ]; then
                log "持续 ${RS485_DETECT_TIMEOUT}s 未收到 RS485 数据"
            else
                log "持续监听时 RS485 串口打开或配置失败 (code=$rs485_rc)"
            fi
        fi
    else
        log "RS485 接收模式不可用，无法执行持续监听"
    fi

    release_rs485_gpio
    set_payload_result "none" ""
    return 0
}

wait_roscore() {
    # === 验证性临时禁用 (2026-08-16)：roslaunch 会自动起 master，不需要等 ===
    # === 若验证通过要彻底删除，恢复原版即可（见下方被注释块） ===
    log "wait_roscore: 已禁用（roslaunch 自动起 master）"
    return 0

    # ----- 原始实现保留如下，需要恢复时取消注释 -----
    # local _
    # for _ in $(seq 1 45); do
    #     rosnode list >/dev/null 2>&1 && { log "roscore ready."; return 0; }
    #     sleep 2
    # done
    # fail "roscore 未就绪 (90s 超时)"
}

wait_ruiyan() {
    # === 验证性临时禁用 (2026-08-16)：multi 节点大多独立于 ruiyan，靠 respawn=true 自愈 ===
    # === 若验证通过要彻底删除，恢复原版即可（见下方被注释块） ===
    log "wait_ruiyan: 已禁用（multi 节点大多独立，靠 respawn=true 自愈）"
    return 0

    # ----- 原始实现保留如下，需要恢复时取消注释 -----
    # local _
    # for _ in $(seq 1 30); do
    #     rosnode list 2>/dev/null | grep -q ruiyan_ros_node && { log "ruiyan_ros_node ready."; return 0; }
    #     sleep 2
    # done
    # fail "ruiyan_ros_node 未就绪 (60s 超时)"
}

# 通用：杀掉某个 roslaunch 父进程及其整棵节点树，并等真退出
kill_launch() {
    local launch_arg="$1"
    local label="$2"
    local pids
    local -a pid_array=()

    # 通过 launch 文件名找所有匹配进程（包括已经 detached 的）
    pids=$(pgrep -f "$launch_arg" 2>/dev/null | tr '\n' ' ')
    if [ -z "$pids" ]; then
        log "$label: no leftover process"
        return 0
    fi
    log "$label: TERM leftover pids: $pids"
    read -r -a pid_array <<< "$pids"
    kill -TERM "${pid_array[@]}" 2>/dev/null || true

    # 等真退出，最多 15s
    local _
    for _ in $(seq 1 15); do
        pids=$(pgrep -f "$launch_arg" 2>/dev/null | tr '\n' ' ')
        [ -z "$pids" ] && return 0
        sleep 1
    done
    # 兜底 KILL
    pids=$(pgrep -f "$launch_arg" 2>/dev/null | tr '\n' ' ')
    if [ -n "$pids" ]; then
        log "$label: KILL stuck pids: $pids"
        read -r -a pid_array <<< "$pids"
        kill -KILL "${pid_array[@]}" 2>/dev/null || true
    fi
    return 0
}

# 通用：启动某个 roslaunch，roslaunch 父进程 PID 写入 PIDFILE，trap 收信号清理
run_launch() {
    local launch_arg="$1"
    local logfile="$2"
    local pidfile="$3"
    local match_arg="${4:-$launch_arg}"
    local require_ruiyan="${5:-true}"
    local release_gpio_on_exit="${6:-false}"
    local -a launch_cmd=()

    log "=== start: roslaunch $launch_arg ==="
    derive_tzs_env
    if ! wait_roscore; then
        [ "$release_gpio_on_exit" = "true" ] && release_rs485_gpio
        return 1
    fi
    if [ "$require_ruiyan" = "true" ]; then
        if ! wait_ruiyan; then
            [ "$release_gpio_on_exit" = "true" ] && release_rs485_gpio
            return 1
        fi
        sleep 3
    fi

    # 清理同名旧进程，并等真退出（关键）
    kill_launch "$match_arg" "pre-start"

    rm -f "$pidfile"
    read -r -a launch_cmd <<< "$launch_arg"
    roslaunch "${launch_cmd[@]}" >"$logfile" 2>&1 &
    local pid=$!
    echo "$pid" > "$pidfile"
    log "roslaunch $launch_arg PID=$pid"

    # systemd Type=simple 下，PID 就是 roslaunch 父进程
    trap 'log "stop signal"; kill -TERM '"$pid"' 2>/dev/null; kill_launch "'"$match_arg"'" "post-stop"; if [ "'"$release_gpio_on_exit"'" = "true" ]; then release_rs485_gpio; fi; rm -f "'"$pidfile"'"; exit 0' TERM INT

    # 死盯 roslaunch 父进程
    while true; do
        if ! kill -0 "$pid" 2>/dev/null; then
            log "roslaunch $launch_arg died, exit so systemd can restart"
            rm -f "$pidfile"
            kill_launch "$match_arg" "post-die"
            if [ "$release_gpio_on_exit" = "true" ]; then
                release_rs485_gpio
            fi
            exit 1
        fi
        sleep 5
    done
}

# =============================================================================
# rosbag 记录：带递增尾号，随 start-multi 启动、stop-multi 停止
# =============================================================================
# 后台等待 multi_uav 节点注册话题后启动 rosbag record，
# 文件名为 <prefix>_<N>.bag（N 从 1 开始，每次启动 +1；--split 时拆成 _N_0.bag）。
start_rosbag_record() {
    sleep "$ROSBAG_START_DELAY"

    # multi_uav launch 已退出则不再启动，避免 stop 后仍拉起孤儿 rosbag
    if [ -f "$MULTI_PIDFILE" ] && kill -0 "$(cat "$MULTI_PIDFILE" 2>/dev/null)" 2>/dev/null; then
        :
    else
        log "multi_uav launch 未运行，跳过 rosbag record"
        return 0
    fi

    mkdir -p "$ROSBAG_BAG_DIR"
    local prefix_="${ROSBAG_BAG_PREFIX}_"
    local max_seq=0
    local f base rest n
    for f in "$ROSBAG_BAG_DIR"/"${prefix_}"*.bag*; do
        [ -e "$f" ] || continue
        base="${f##*/}"
        base="${base%.bag.active}"
        base="${base%.bag}"
        rest="${base#"$prefix_"}"
        n="${rest%%_*}"
        case "$n" in
            ''|*[!0-9]*) continue ;;
        esac
        [ "$n" -gt "$max_seq" ] && max_seq="$n"
    done
    local next_seq=$((max_seq + 1))
    local out="${ROSBAG_BAG_DIR}/${ROSBAG_BAG_PREFIX}_${next_seq}.bag"
    log "rosbag record → ${out}"

    rosbag record -O "$out" -a --split --size=2048 --buffsize=1024 \
        -x "$ROSBAG_EXCLUDE_REGEX" >>"$LOG_DIR/rosbag_record.log" 2>&1 &
    echo $! > "$ROSBAG_PIDFILE"
    log "rosbag record PID=$!"
}

stop_rosbag_record() {
    if [ -f "$ROSBAG_PIDFILE" ]; then
        local pid; pid=$(cat "$ROSBAG_PIDFILE" 2>/dev/null)
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            # rosbag 收到 SIGINT 才会 finalize 当前 bag
            kill -INT "$pid" 2>/dev/null || true
            for _ in $(seq 1 8); do
                kill -0 "$pid" 2>/dev/null || break
                sleep 1
            done
            kill -KILL "$pid" 2>/dev/null || true
        fi
        rm -f "$ROSBAG_PIDFILE"
    fi
}

run_payload_idle() {
    local state="$1"

    kill_launch "payload_autostart payload_autostart.launch" "pre-idle"
    echo "$$" > "$PAYLOAD_PIDFILE"
    log "载荷状态为 $state，无载荷节点需要启动；服务保持运行"

    trap 'log "stop signal"; release_rs485_gpio; rm -f "'"$PAYLOAD_PIDFILE"'"; exit 0' TERM INT
    while true; do
        sleep 30
    done
}

# 没有载荷时，把 payload_type 推给飞控。
# /uav0/system/payload_type 的订阅端一般在 multi_uav_strike 节点里，
# 这里等最多 30s 让 roscore 起来再发，避免单次 pub 在订阅端上线之前丢失。
publish_no_payload_topic() {
    local value="${1:-7}"
    local topic="/uav0/system/payload_type"
    local msg_type="std_msgs/UInt8"

    log "未识别到载荷：准备发布 $topic ($msg_type, data=$value)"

    local _
    for _ in $(seq 1 30); do
        if rostopic list >/dev/null 2>&1; then
            break
        fi
        sleep 1
    done

    if ! rostopic list >/dev/null 2>&1; then
        log "WARN: roscore 30s 内未就绪，跳过 $topic 发布（data=$value）"
        return 1
    fi

    if rostopic pub -l "$topic" "$msg_type" "data: $value" -1; then
        log "$topic 已发布一次 payload_type=$value"
    else
        log "WARN: $topic 发布失败 (rc=$?)"
        return 1
    fi
}

start_payload() {
    log "TZS_IMAGE_VERSION=$TZS_IMAGE_VERSION"
    derive_tzs_env
    detect_payload

    case "$TZS_PAYLOAD_TYPE" in
        lidar|relay|rs485)
            local launch_arg="payload_autostart payload_autostart.launch payload_type:=$TZS_PAYLOAD_TYPE payload_endpoint:=$TZS_PAYLOAD_ENDPOINT"
            run_launch "$launch_arg" "$LOG_DIR/payload_autostart.log" "$PAYLOAD_PIDFILE" \
                "payload_autostart payload_autostart.launch" false true
            ;;
        none)
            publish_no_payload_topic 7
            run_payload_idle "$TZS_PAYLOAD_TYPE"
            ;;
        conflict)
            run_payload_idle "$TZS_PAYLOAD_TYPE"
            ;;
    esac
}
start_multi() {
    log "TZS_IMAGE_VERSION=$TZS_IMAGE_VERSION"
    # 后台：等 multi_uav 节点注册话题后启动 rosbag record
    start_rosbag_record &
    run_launch "multi_uav_strike single_uav_px4_flight_test.launch" "$LOG_DIR/multi_uav_strike.log" "$MULTI_PIDFILE"
}

stop_one() {
    local pidfile="$1"
    local launch_arg="$2"
    if [ -f "$pidfile" ]; then
        local pid; pid=$(cat "$pidfile" 2>/dev/null)
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
            for _ in $(seq 1 15); do
                kill -0 "$pid" 2>/dev/null || break
                sleep 1
            done
            kill -KILL "$pid" 2>/dev/null || true
        fi
        rm -f "$pidfile"
    fi
    kill_launch "$launch_arg" "explicit-stop"
}

stop_payload() {
    stop_one "$PAYLOAD_PIDFILE" "payload_autostart payload_autostart.launch"
    release_rs4
    stop_one "$MULTI_PIDFILE"  "multi_uav_strike single_uav_px4_flight_test.launch"
    stop_rosbag_record

}
stop_multi()  { stop_one "$MULTI_PIDFILE"  "multi_uav_strike single_uav_px4_flight_test.launch"; }

case "${1:-}" in
    start-payload|start_payload|start-gimbal|start_gimbal) shift; start_payload ;;
    start-multi|start_multi)   shift; start_multi  ;;
    stop-payload|stop_payload|stop-gimbal|stop_gimbal) shift; stop_payload ;;
    stop-multi|stop_multi)     shift; stop_multi   ;;
    show-env|show_env)
        derive_tzs_env
        echo "TZS_ETH0_IP=$TZS_ETH0_IP"
        echo "TZS_DEVICE_SN=$TZS_DEVICE_SN"
        echo "TZS_FCU_ID=$TZS_FCU_ID"
        echo "TZS_PAYLOAD_TYPE=${TZS_PAYLOAD_TYPE:-none}"
        echo "TZS_PAYLOAD_ENDPOINT=${TZS_PAYLOAD_ENDPOINT:-}"
        echo "(已写入 $ENV_FILE, 可用: source $ENV_FILE)"
        ;;
    start|stop|restart)
        log "用法已变化：请用 start-payload/start-multi/stop-payload/stop-multi/show-env"
        log "不要直接调用 start/stop/restart，由 systemd 拉起"
        exit 1
        ;;
    *) echo "Usage: $0 {start-payload|start-multi|stop-payload|stop-multi|show-env}"; exit 1 ;;
esac
