#!/usr/bin/env bash
# gs_replay_random.sh
#
# 自动化回归测试:启动 SITL + 全 UAV 节点栈,逐个播放 GS 录制 bag,
# 录制每只 bag 的完整系统反应 (mavros/state 等),跑完一轮后分析每只
# 回放 bag 中的 mavros/state.mode,只要出现过 AUTO.LAND 就认为此场景
# 出问题,默认停下来报告并退出,不在它之后继续后面的 bag。
#
# 一次性源进 catkin_ws,然后:
#   scripts/gs_replay_random.sh --loop N
#
# 注意:
#   - 不必自己启 SITL,脚本默认在第一次循环前启 ~/src/PX4_Firmware/Tools/sitl_single_run.sh,
#     走完所有 loop 后杀;若已自己启 SITL,加 --no-start-sitl。
#   - analyze 阶段会用 rosbag (Python) 解析播放 bag,只读 mavros/state 一个话题,
#     时间 < 1s。
#   - 回放 bag 落盘位置默认 ~/LOG/gs_playback_recorded.bag (单只复用,每轮覆盖)。
#     若不希望覆盖,用 --playback-bag-name=gs_playback_<ts> 自带时间戳。

set -uo pipefail

# ----------------------------- 默认参数 -----------------------------
BAG_DIR="${HOME}/LOG"
BAG_PREFIX="gs_test_play_"
LOOP=1
RATE=1.0
STARTUP_DELAY=12       # roslaunch 后 Mavros 多握手几秒
POST_DELAY=5           # bag 走完再多等几秒再 SIGINT
UAV_NAME="uav0"

# SITL 控制
START_SITL=1
SITL_SCRIPT="${HOME}/src/PX4_Firmware/Tools/sitl_single_run.sh"
SITL_WAIT=22           # SITL+PX4+jMAVSim 启动到 MAVROS 能连上的实测时长
SITL_LOG="/tmp/gs_replay_sitl.log"

# 回放录制
RECORD_PLAYBACK_BAG=1
PLAYBACK_BAG_NAME="gs_playback_recorded"
NO_RECORD_PLAYBACK_BAG_FLAG=""   # 占位,实际通过 RECORD_PLAYBACK_BAG 控制

# 分析
STATE_TOPIC="/uav0/mavros/state"      # 也支持 glob:'-uav0/mavros/state'
AUTO_LAND_STOP=1                      # 1=出问题立即停
REPORT_FILE=""                        # 空则写 /tmp/gs_replay_report.txt
DRY_RUN=0

# ----------------------------- 用法 -----------------------------
usage() {
    cat <<'USAGE'
用法: gs_replay_random.sh [opts]

完成动作 (SITL 每只 bag 之前都重起,analyze 期间停):
  for i in 1..LOOP:
      ① 启 SITL (sitl_single_run.sh) → ② 等 SITL_WAIT 秒
      ③ 随机挑 ${BAG_DIR}/${BAG_PREFIX}*.bag
      ④ roslaunch multi_uav_strike gs_play.launch 喂此 bag + 同步全录成
         ~/LOG/${PLAYBACK_BAG_NAME}.bag
      ⑤ 等 STARTUP_DELAY + duration/RATE + POST_DELAY 秒, SIGINT 退出
      ⑥ 杀掉 SITL/jMAVSim (资源干净)
      ⑦ 扫播放 bag 中 mavros/state.mode, 出现 AUTO.LAND ⇒ AUTO_LAND_STOP=1 则
         停下,否则继续
  ⑧ 全部跑完 / 提前 FAIL, 输出报告

可选:
  --bag-dir DIR             检索根目录 (default: $HOME/LOG)
  --bag-prefix PREFIX       bag 文件名前缀 (default: gs_test_play_)
  --loop N                  连跑 N 个 bag (default: 1)
  --rate R                  回放倍率 (default: 1.0)
  --startup-delay S         roslaunch 后多等 S 秒 (default: 12)
  --post-delay S            bag 走完再多等 S 秒再 SIGINT (default: 5)
  --uav-name NAME           UAV namespace (default: uav0)
  --start-sitl / --no-start-sitl
                            默认启 SITL; --no-start-sitl 时由用户自己启
                            (脚本每只 bag 之前会重起 SITL,analyze 期间关掉)
  --sitl-script PATH        SITL 脚本路径 (default: ~/src/PX4_Firmware/Tools/sitl_single_run.sh)
  --sitl-wait S             SITL 起来后等多秒再进 loop (default: 22)
  --record-playback-bag / --no-record-playback-bag
                            默认同步全录一只 playback bag
  --playback-bag-name NAME  playback bag 文件名不带 .bag 后缀 (default: gs_playback_recorded)
  --state-topic TOPIC       mavros state 话题,支持 *,逗号分隔
                            (default: /uav0/mavros/state)
  --auto-land-stop / --no-auto-land-stop
                            默认 AUTO.LAND 出现即停 (default: ON)
  --report-file PATH        FAIL 报告输出路径 (default: /tmp/gs_replay_report.txt)
  --dry-run                 仅列候选 bag
  -h|--help                 this help
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --bag-dir)              BAG_DIR="$2"; shift 2 ;;
        --bag-prefix)           BAG_PREFIX="$2"; shift 2 ;;
        --loop)                 LOOP="$2"; shift 2 ;;
        --rate)                 RATE="$2"; shift 2 ;;
        --startup-delay)        STARTUP_DELAY="$2"; shift 2 ;;
        --post-delay)           POST_DELAY="$2"; shift 2 ;;
        --uav-name)             UAV_NAME="$2"; shift 2 ;;
        --start-sitl)           START_SITL=1; shift ;;
        --no-start-sitl)        START_SITL=0; shift ;;
        --sitl-script)          SITL_SCRIPT="$2"; shift 2 ;;
        --sitl-wait)            SITL_WAIT="$2"; shift 2 ;;
        --record-playback-bag)  RECORD_PLAYBACK_BAG=1; shift ;;
        --no-record-playback-bag) RECORD_PLAYBACK_BAG=0; shift ;;
        --playback-bag-name)    PLAYBACK_BAG_NAME="$2"; shift 2 ;;
        --state-topic)          STATE_TOPIC="$2"; shift 2 ;;
        --auto-land-stop)       AUTO_LAND_STOP=1; shift ;;
        --no-auto-land-stop)    AUTO_LAND_STOP=0; shift ;;
        --report-file)          REPORT_FILE="$2"; shift 2 ;;
        --dry-run)              DRY_RUN=1; shift ;;
        -h|--help)              usage; exit 0 ;;
        *) echo "[gs_replay_random] unknown opt: $1" >&2; usage; exit 2 ;;
    esac
done
[[ -z "$REPORT_FILE" ]] && REPORT_FILE="/tmp/gs_replay_report.txt"

# ----------------------------- 工具函数 -----------------------------

# 取 bag 时长(秒),用 --yaml 第二行的 duration: <float>
get_duration() {
    local bag="$1"
    rosbag info --yaml "$bag" 2>/dev/null \
        | python3 -c '
import sys
for line in sys.stdin:
    line = line.strip()
    if line.startswith("duration:"):
        try:
            print(float(line.split(":", 1)[1].strip()))
            sys.exit(0)
        except ValueError:
            pass
print("-1", file=sys.stderr); sys.exit(2)
'
}

# 分析播放 bag:扫 mavros/state 看有没有 mode=AUTO.LAND
# stdout: 报告; stderr: 错误; exit: 0 OK / 1 FAIL / 2 错误
analyze_playback_bag() {
    local bag="$1"
    [[ ! -f "$bag" ]] && { echo "ERROR: bag not found: $bag" >&2; return 2; }
    python3 - "$bag" << 'PYEOF'
import sys
from rosbag import Bag
bag_file = sys.argv[1]
land_events = []
state_seen = 0
with Bag(bag_file, "r") as bag:
    for topic, msg, t in bag.read_messages():
        if not (topic == "/mavros/state" or topic.endswith("/mavros/state")):
            continue
        state_seen += 1
        mode = getattr(msg, "mode", None)
        if not isinstance(mode, str):
            continue
        if mode == "AUTO.LAND":
            land_events.append((t.to_sec(), topic, mode))

if not land_events:
    print(f"OK: scanned {state_seen} mavros/state msgs, no AUTO.LAND in {bag_file}")
    sys.exit(0)
print(f"FAIL: AUTO.LAND detected {len(land_events)} timestamp(s) in {bag_file}")
for ts, topic, mode in land_events[:30]:
    print(f"  t={ts:.3f}s  topic={topic}  mode={mode}")
if len(land_events) > 30:
    print(f"  ... ({len(land_events) - 30} more)")
sys.exit(1)
PYEOF
}

# ----------------------------- SITL 生命周期 -----------------------------
SITL_PID=""

start_sitl() {
    [[ "$START_SITL" -eq 0 ]] && { echo "[gs_replay_random] --no-start-sitl, 跳过启 SITL"; return; }
    if [[ ! -f "$SITL_SCRIPT" ]]; then
        echo "[gs_replay_random] WARN: SITL script not found: $SITL_SCRIPT (跳过启 SITL)" >&2
        return
    fi
    echo "[gs_replay_random] starting SITL via setsid: $SITL_SCRIPT"
    # setsid 把 sitl_single_run.sh 自立成 session+pgid leader,
    #   后面 SIGKILL -- -PID 才能把 bash + jMAVSim + PX4 整组干掉,
    #   否则 SIGINT 单发到 bash,bash 死掉,jMAVSim/PX4 被 init 收养继续跑。
    setsid bash "$SITL_SCRIPT" > "$SITL_LOG" 2>&1 < /dev/null &
    SITL_PID=$!
    disown "$SITL_PID" 2>/dev/null || true
    echo "[gs_replay_random] SITL pid=$SITL_PID (group/session leader) log=$SITL_LOG"
    echo "[gs_replay_random] waiting ${SITL_WAIT}s for PX4+jMAVSim+MAVROS to come up..."
    sleep "$SITL_WAIT"
}

stop_sitl() {
    [[ -z "$SITL_PID" ]] && return
    echo "[gs_replay_random] killing SITL process group (pgid=$SITL_PID)"
    # 1) SIGINT 整组 (best effort, 让 jMAVSim Java ShutdownHook 自清)
    kill -INT -- "-$SITL_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8; do
        kill -0 "$SITL_PID" 2>/dev/null || break
        sleep 1
    done
    # 2) SIGKILL 整组兜底
    kill -KILL -- "-$SITL_PID" 2>/dev/null || true
    # 3) 兜底之上再加兜底:某些 jMAVSim 派生可能跳出组,用名字收
    pkill -KILL -f "jmavsim_run\.sh|jMAVSim\.jar|jmavsim" 2>/dev/null || true
    # 4) SITL_PID 本体
    kill -KILL "$SITL_PID" 2>/dev/null || true
    wait "$SITL_PID" 2>/dev/null || true
    SITL_PID=""
}

# ----------------------------- roslaunch 清理 -----------------------------
ROSLAUNCH_PID=""

cleanup() {
    if [[ -n "$ROSLAUNCH_PID" ]] && kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
        echo "[gs_replay_random] SIGINT roslaunch_pid=$ROSLAUNCH_PID"
        kill -INT "$ROSLAUNCH_PID" 2>/dev/null
        for _ in 1 2 3 4 5; do
            kill -0 "$ROSLAUNCH_PID" 2>/dev/null || break
            sleep 1
        done
        kill -KILL "$ROSLAUNCH_PID" 2>/dev/null || true
        wait "$ROSLAUNCH_PID" 2>/dev/null || true
    fi
}
trap 'cleanup; stop_sitl; trap - EXIT' EXIT INT TERM

# ----------------------------- dry-run -----------------------------
if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "[gs_replay_random] DRY-RUN:"
    echo "  scan:  ${BAG_DIR}/${BAG_PREFIX}*.bag"
    echo "  sitl:  $([[ $START_SITL -eq 1 ]] && echo "auto-start $SITL_SCRIPT (wait ${SITL_WAIT}s)" || echo "skipped (--no-start-sitl)")"
    echo "  rec:   $([[ $RECORD_PLAYBACK_BAG -eq 1 ]] && echo "${BAG_DIR}/${PLAYBACK_BAG_NAME}.bag (full topics)" || echo "off")"
    echo "  state-topic:  $STATE_TOPIC"
    echo "  stop-on-AUTO.LAND: $([[ $AUTO_LAND_STOP -eq 1 ]] && echo yes || echo no)"
    echo "  loop:  $LOOP"
    echo ""
    find "$BAG_DIR" -maxdepth 1 -name "${BAG_PREFIX}*.bag" -type f 2>/dev/null
    exit 0
fi

# ----------------------------- 主循环 -----------------------------
: > "$REPORT_FILE"  # 清空旧报告

PLAYBACK_BAG_PATH="$BAG_DIR/$PLAYBACK_BAG_NAME.bag"
STOPPED=0
RUNNING=1

for i in $(seq 1 "$LOOP"); do
    BAG=$(find "$BAG_DIR" -maxdepth 1 -name "${BAG_PREFIX}*.bag" -type f 2>/dev/null \
          | shuf -n 1)
    if [[ -z "${BAG:-}" ]]; then
        echo "[gs_replay_random] ERR: no ${BAG_PREFIX}*.bag in $BAG_DIR" >&2
        exit 1
    fi
    BAG_NAME=$(basename "$BAG")
    echo ""
    echo "===================================================================="
    echo "[gs_replay_random] [$i/$LOOP] $(date +%T) pick=$BAG_NAME"

    # ---- 0) 清掉上一轮残留的 playback bag,避免 .bag.active / 冲突文件名 ----
    shopt -s nullglob
    for stale in "${BAG_DIR}/${PLAYBACK_BAG_NAME}".bag \
                 "${BAG_DIR}/${PLAYBACK_BAG_NAME}".bag.active; do
        [[ -f "$stale" ]] && rm -f "$stale"
    done

    # ---- 1) 启 SITL (fresh,每个 bag 之前一次) ----
    start_sitl

    DURATION_S=$(get_duration "$BAG" 2>/dev/null)
    if [[ -z "$DURATION_S" || "$DURATION_S" == "-1" ]]; then
        echo "[gs_replay_random] ERR: cannot parse duration from $BAG" >&2
        exit 1
    fi
    PLAY_TIME=$(python3 -c "import math; print(int(math.ceil(${DURATION_S}/${RATE} + ${POST_DELAY})))")
    TOTAL=$(( STARTUP_DELAY + PLAY_TIME ))
    echo "    duration=${DURATION_S}s  rate=${RATE}  startup=${STARTUP_DELAY}s  play=${PLAY_TIME}s  total=${TOTAL}s"
    echo "    playback_bag=$PLAYBACK_BAG_PATH"

    ROSLAUNCH_LOG="/tmp/gs_replay.$(date +%s).log"
    # 同步录制选项透传 (默认开启)
    LP_FLAGS=()
    [[ "$RECORD_PLAYBACK_BAG" -eq 1 ]] && LP_FLAGS=()
    [[ "$RECORD_PLAYBACK_BAG" -eq 0 ]] && LP_FLAGS=("record_playback_bag:=false")

    # shellcheck disable=SC2086
    roslaunch multi_uav_strike gs_play.launch \
        uav_name:="$UAV_NAME" \
        bag_path:="$BAG" \
        bag_dir:="$BAG_DIR" \
        play_rate:="$RATE" \
        playback_bag_name:="$PLAYBACK_BAG_NAME" \
        "${LP_FLAGS[@]}" \
        > "$ROSLAUNCH_LOG" 2>&1 &
    ROSLAUNCH_PID=$!
    echo "    [$(date +%T)] roslaunch_pid=$ROSLAUNCH_PID log=$ROSLAUNCH_LOG"

    echo "    [$(date +%T)] waiting ${TOTAL}s total..."
    sleep "$TOTAL"

    echo "    [$(date +%T)] run $i/$LOOP done. SIGINT roslaunch and stop SITL before analyze."
    cleanup

    # ---- 2) analyze 阶段把 SITL/jMAVSim 关掉,资源干净,下一只 bag 重启 ----
    stop_sitl

    echo "    ----- tail $ROSLAUNCH_LOG -----"
    tail -8 "$ROSLAUNCH_LOG" | sed 's/^/    /'

    echo "    ----- analyze $PLAYBACK_BAG_PATH -----"
    ANALYZE_OUT=$(analyze_playback_bag "$PLAYBACK_BAG_PATH" 2>&1)
    ANALYZE_RC=$?
    echo "$ANALYZE_OUT" | sed 's/^/    /'
    echo "    [analyze rc=$ANALYZE_RC]"

    # 写报告
    {
        echo "=========================================================="
        echo "loop $i/$LOOP @ $(date +%T)"
        echo "input_bag:    $BAG"
        echo "playback_bag: $PLAYBACK_BAG_PATH"
        echo "state_topic:  $STATE_TOPIC"
        echo "rc: $ANALYZE_RC (0=ok, 1=FAIL, 2=error)"
        echo "$ANALYZE_OUT"
    } >> "$REPORT_FILE"

    if [[ "$ANALYZE_RC" -eq 1 && "$AUTO_LAND_STOP" -eq 1 ]]; then
        echo ""
        echo "===================================================================="
        echo "[gs_replay_random] ABORT: AUTO.LAND detected → stop further loops."
        echo "  failed_input_bag:    $BAG"
        echo "  playback_bag:        $PLAYBACK_BAG_PATH  (mavros/state 含 AUTO.LAND)"
        echo "  report:              $REPORT_FILE"
        echo "===================================================================="
        STOPPED=1
        RUNNING=0
        break
    fi
done

echo ""
echo "===================================================================="
if [[ "$STOPPED" -eq 1 ]]; then
    echo "[gs_replay_random] ABORTED after FAIL. report: $REPORT_FILE"
else
    echo "[gs_replay_random] all $LOOP run(s) finished cleanly. report: $REPORT_FILE"
fi
trap - EXIT
exit 0
