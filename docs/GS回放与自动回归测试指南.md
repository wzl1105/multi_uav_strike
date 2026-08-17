# GS 回放与自动回归测试指南

> 用 `rosbag record/play` 把"地面站发出过什么命令 / 系统怎么反应"录下来再喂回去，
> 摆脱"必须有真 GS 才能跑回归"的束缚。
>
> 三件套:
>
> - [`launch/gs_record.launch`](#21-gs_recordlaunch-录制) —— 单只录制,启动 SITL + 全 UAV 节点栈 + 白名单录 `starling_bridge` 的 18 个发布
> - [`launch/gs_play.launch`](#22-gs_playlaunch-回放) —— 单只回放,把录好的 bag 喂回系统,**同步全录**一只 playback bag
> - [`scripts/gs_replay_random.sh`](#23-gs_replay_randomsh-批量自动回归) —— 批量跑 + AUTO.LAND 分析 + FAIL 停栈
>
> 适用场景:
>
> - 调参后想回归"GS 推 flow → UAV 完成搜索打击"的链路
> - 改完 `starling_bridge`/`mission_manager` 后的回归
> - CI / 值班跑回归集,出问题立即停了不上跑后面的

---

## 1. 三件套的关系

```
┌─────────────────────────┐
│   gs_record.launch      │   ← 录制:白名单 18 个 starling_bridge 发布
│   (SITL + UAV 栈)       │
│   产物 gs_test_play_X.bag│
└────────────┬────────────┘
             │ 单或多只
             ▼
┌─────────────────────────┐
│   gs_play.launch        │   ← 回放 + 全录
│   SITL + UAV 栈          │   输出 ~/LOG/gs_playback_recorded.bag
│   ✓ rosbag play          │
│   ✓ rosbag record -a     │
└────────────┬────────────┘
             │ 一只 playback bag
             ▼
┌─────────────────────────┐    FAIL: AUTO.LAND 在 mavros/state 出现
│   gs_replay_random.sh   │ ─────────────────────────────────────► 停
│   (批量 random + analyze)│
└─────────────────────────┘
```

三个文件互相依赖,任何一只可单独跑(给 `--dry-run` / 手动指定 bag),组合起来就是「批量自动化」。

---

## 2. 单只流程

### 2.1 `gs_record.launch` 录制

#### 节点清单

与 `single_uav_px4_tzs_ctl.launch` 节点清单完全一致,只是 tzs_ctl 里 3 段云台模拟节点被注释掉了,
gs_record / gs_play **主动 include 它**(顶层 include,节点落在 `/uav0/gimbal_simulator_node`,
不嵌套 ns)。两launch比 tzs_ctl 多一个 node:`/uav0/gimbal_simulator_node`。

只把第 11 段 rosbag 的"全录 + 排除图像"换成 "只录 `starling_bridge` 发布的 18 个话题":

| 来源 | 话题 |
|---|---|
| `uav0` 命名空间(15 个) | `tzs/mission/ctl`、`mission/{task_flow,work_mode,attack_cmd}`、`mavlink/to`、`mavros/manual_control/send`、`tzs/cmd/{media_switch,set_mode,manual_control,gimbal_angular_velocity}`、`tzs/monitor/target_snapshot`、`tzs/state/{launch_intent,launch_state}` |
| 绝对话题(3 个) | `/gs/{mode_cmd,waypoint_upload}`、`/tzs/media_switch`、`/gimbal/media_switch`、`/inter_uav/uav_status` |

**rosbag record 不支持 `-k` inclusion regex(Noetic 上会报 `unrecognised option '-k'`)**,
所以这 18 个话题是**作为位置参数逐个传给 `rosbag record`** 的,见
[gs_record.launch:53](/home/jack/catkin_ws/src/multi_uav_strike/launch/gs_record.launch#L53)。

#### 手动录制

```bash
# 终端 1
jack@jack-y9000k:~/src/PX4_Firmware/Tools$ ./sitl_single_run.sh

# 终端 2
cd ~/catkin_ws && source devel/setup.bash
roslaunch multi_uav_strike gs_record.launch bag_name:=gs_test_play_1
# 产物: ~/LOG/gs_test_play_1.bag
# 跑一会儿,Ctrl+C 结束(rosbag 自动 finalize 当前 bag)
```

为了让回放端能拿到完整 GS 命令序列,**建议在 SITL 完全起来 + SITL 推到 task_flow 之后再开始录**。
也可以给 `rosbag record` 加 `--duration 120` 这种硬截止时间。

#### 重命名 bag

```bash
roslaunch multi_uav_strike gs_record.launch bag_name:=gs_test_play_2
roslaunch multi_uav_strike gs_record.launch bag_name:=gs_test_play_3
...
```

`bag_name` 不带 `.bag` 后缀,落盘 `<bag_dir>/<bag_name>.bag`(默认 `~/LOG/`)。

#### `gz` 节点不影响

`tgt_system / gcs_url / gcs_ip` 这些参数沿用 tzs_ctl 默认,手动跑不需要改。
如果接的是别台机器,`TZS_FCU_ID` / `TZS_GCS_IP` 环境变量覆盖即可。

### 2.2 `gs_play.launch` 回放

#### 节点清单

同 `gs_record.launch`(包括 gimbal_simulator_node 的 include),加两个额外动作:

| 节点 | 作用 |
|---|---|
| `starling_pub_rosbag_play` | `rosbag play <bag> -r <rate>`,把录制时 `starling_bridge` 发出的命令重新发回 ROS |
| `playback_full_bag_record` | `rosbag record -a`,**全录**系统这一轮的反应(默认带 `/uav0/mavros/state`,便于 analyze) |

**`/uav0/gimbal_simulator_node` 的生命周期**:通过 roslaunch include 进来,
所以 `gs_replay_random.sh` 里的 SIGINT roslaunch 会把它和 Mavros / mission_manager 等一起关掉。
如果你想"明确独立的 gimbal launch 生命周期"(像 SITL 那样 setsid + 整组 kill),
告诉我,加 `--enable-gimbal / --gimbal-launch` flag。

#### 手动回放

```bash
# SITL 同上,跑起来

cd ~/catkin_ws && source devel/setup.bash
roslaunch multi_uav_strike gs_play.launch \
    bag_path:=$HOME/LOG/gs_test_play_1.bag \
    play_rate:=1.0
# 等 bag 走完,Ctrl+C
```

产物:`~/LOG/gs_playback_recorded.bag`(playback bag,含全部话题)。

> **注意时序**:bag 从 roslaunch 那一刻(t=0)就开始回放了,但 SITL+Mavros 真正握手完成大概要 8~12s,
> 前 ~10s 的 GS 命令很可能落在 Mavros 还没订阅完的窗口里被吞。**解决方法**:录制时也别从 t=0 开始录,
> 等到 SITL 起来、`gs_simulator_node.py` 已经推完首条 typed TaskFlow 之后 Ctrl+R(先停再录)再启动
> rosbag,或者下版给 `gs_play.launch` 加个 `launch-prefix` + `startup_delay` arg。

#### 只关 playback record

```bash
roslaunch multi_uav_strike gs_play.launch \
    bag_path:=$HOME/LOG/gs_test_play_1.bag \
    record_playback_bag:=false   # 不录 playback bag
```

### 2.3 `gs_replay_random.sh` 批量自动回归

把上面的"启动 SITL → 等一下 → 选 bag → 跑 → 杀 → 分析"全揉在一只脚本里,
可批量跑、可随机挑 bag、可检测 AUTO.LAND 提前停。

#### 调用模板

```bash
cd ~/catkin_ws && source devel/setup.bash

# 最常用:启 SITL 自动,跑 5 只
./src/multi_uav_strike/scripts/gs_replay_random.sh --loop 5

# SITL 已自己启了,脚本只做播放+分析
./src/multi_uav_strike/scripts/gs_replay_random.sh --no-start-sitl --loop 5

# 2x 加速,playback bag 留时间戳,不复用一只
./src/multi_uav_strike/scripts/gs_replay_random.sh --rate 2.0 \
    --playback-bag-name gs_playback_$(date +%H%M%S)
```

#### 一次性流程(SITL **每只 bag** 前重起)

```
loop i in 1..N:
   ① 启 SITL (sitl_single_run.sh) → ② 等 SITL_WAIT 秒
   ③ 随机挑 ${BAG_DIR}/${BAG_PREFIX}*.bag
   ④ roslaunch gs_play.launch bag_path:=BAG playback_bag_name:=P
   ⑤ 等 STARTUP_DELAY + duration/RATE + POST_DELAY 秒
   ⑥ SIGINT roslaunch → ⑦ 杀掉 SITL / jMAVSim(analyze 阶段不占资源)
   analyze ~/LOG/<P>.bag:
     用 rosbag (Python) 扫 /uav0/mavros/state
     任何 message.mode == "AUTO.LAND"  →  FAIL
   FAIL && AUTO_LAND_STOP=1 → 写出报告、退出 loop
                     else    → 回到 ① 重起 SITL,下一只 bag
Ctrl+C / 异常退出 → trap 把遗留 SITL 也清掉
```

**重要**:每只 bag 之前 SITL 都重起,所以每轮 PX4 都从同一初始状态开始(原 `home_lat/lon/alt`)。

如果想保留旧行为(整批共用一只 SITL,只在脚本退出时杀掉),可以用 `kill -INT $(pgrep -f sitl_single_run)` 后手动跑 `--no-start-sitl`(但当前版本里没单独留 flag,可以加 `--keep-sitl-across-iters`)。

#### 输出物

| 文件 | 内容 |
|---|---|
| `~/LOG/gs_playback_recorded.bag` | 每轮覆盖式全录 playback 结果(若 `--playback-bag-name` 自定义则按名字) |
| `/tmp/gs_replay.<ts>.log` | 每轮 roslaunch 的完整 stdout,失败时 `tail` 用 |
| `/tmp/gs_replay_sitl.log` | SITL(`sitl_single_run.sh`)的 stdout |
| `/tmp/gs_replay_report.txt` | **每轮 analyze 的快照**,input bag、playback bag、analyze rc、原文输出 |

#### FAIL 时长啥样

```
[gs_replay_random] [3/5] 14:22:01 pick=gs_test_play_3.bag
    duration=104.6s  rate=1.0  startup=12s  play=110s  total=122s
    playback_bag=/home/jack/LOG/gs_playback_recorded.bag
    ----- analyze /home/jack/LOG/gs_playback_recorded.bag -----
    FAIL: AUTO.LAND detected 4 timestamp(s) in /home/jack/LOG/gs_playback_recorded.bag
      t=42.180s  topic=/uav0/mavros/state  mode=AUTO.LAND
      t=44.360s  topic=/uav0/mavros/state  mode=AUTO.LAND
      t=48.000s  topic=/uav0/mavros/state  mode=AUTO.LAND
      t=58.420s  topic=/uav0/mavros/state  mode=AUTO.LAND
    [analyze rc=1]
[gs_replay_random] ABORT: AUTO.LAND detected → stop further loops.
  failed_input_bag:    /home/jack/LOG/gs_test_play_3.bag
  playback_bag:        /home/jack/LOG/gs_playback_recorded.bag  (mavros/state 含 AUTO.LAND)
  report:              /tmp/gs_replay_report.txt
```

报告(`/tmp/gs_replay_report.txt`)的格式:

```
==========================================================
loop 3/5 @ 14:22:01
input_bag:    /home/jack/LOG/gs_test_play_3.bag
playback_bag: /home/jack/LOG/gs_playback_recorded.bag
state_topic:  /uav0/mavros/state
rc: 1 (0=ok, 1=FAIL, 2=error)
FAIL: AUTO.LAND detected 4 timestamp(s) ...
  t=42.180s  topic=/uav0/mavros/state  mode=AUTO.LAND
  ...
```

#### 关闭 AUTO.LAND 停栈

```bash
# 跑完所有 loop 不停,即使中间某只撞 AUTO.LAND
./src/multi_uav_strike/scripts/gs_replay_random.sh --loop 5 --no-auto-land-stop
# 看报告 /tmp/gs_replay_report.txt 里哪几只 rc=1 即可
```

#### 完全不录 playback bag(只分析残跑过的)

```bash
./src/multi_uav_strike/scripts/gs_replay_random.sh --no-record-playback-bag --loop 5
# analyze 会因找不到 playback bag 直接 rc=2 报 ERROR;这种情况一般配合
# --no-auto-land-stop 用,或者干脆手动跑 gs_play.launch
```

---

## 3. 关键概念

### 3.1 录制白名单 vs 回放全录

| 阶段 | 录什么 | 为什么 |
|---|---|---|
| `gs_record.launch` | **白名单 18 个 `starling_bridge` 发布** | 只关心"GS 看过什么 / 会发什么命令",单只 bag 几十 KB,随便录多次不爆盘 |
| `gs_play.launch` | **全录 `-a` 包括 `mavros/state`** | analyze 阶段要的是系统反应(状态机切换、模式变化),`mavros/state` 不能丢 |

### 3.2 SITL + Mavros 的启动时序

| 阶段 | 实际耗时 |
|---|---|
| `sitl_single_run.sh` 启 PX4 + jMAVSim | 15~25s(取决于机器) |
| 第一次 rosmaster + Mavros handshake | 8~12s |
| Mavros 收到首帧 `mavros/state.connected=true` | 累计 ~30s |

`gs_replay_random.sh` 把这三层都吃进来了:`--sitl-wait`(默认 22s)只覆盖前一段,
剩下两段由 `--startup-delay`(默认 12s)在 roslaunch 后再补。

### 3.3 每轮 SITL 重起的代价

每轮 SITL 都要重起(`setsid bash sitl_single_run.sh & sleep SITL_WAIT`),所以 `--loop N` 的
总时长 ≈ `N × ( SITL_WAIT(22) + STARTUP_DELAY(12) + duration/rate + POST_DELAY(5) )`。
以 5 只、平均 100s 的 bag 为例,**5 轮的总时长 ≈ 5×(22+12+100+5) ≈ 11 分钟**。
如果嫌长,把 `--loop` 改小或用 `--no-start-sitl` 自己启 SITL。

### 3.4 SITL 怎么确保被杀掉

`sitl_single_run.sh` 自己 bash 起 PX4,然后 `Tools/jmavsim_run.sh -l` 起 jMAVSim(Java GUI)。
普通 `bash script.sh &; kill $!` 只能杀掉 bash 父脚本,jMAVSim **被 init 收养继续跑**
(bash 默认不会把信号级联给子进程,Java ShutdownHook 也不一定响应 SIGINT)。

修法:

1. **`setsid bash sitl_single_run.sh &`** —— 启动时把整个树自立成新 session+pgid leader,
   SITL_PID 就是 pgid
2. **`kill -KILL -- "-$SITL_PID"`** —— 负号=整组,把 bash + jMAVSim + PX4 一次性 SIGKILL
3. **兜底 `pkill -KILL -f "jmavsim_run\.sh|jMAVSim\.jar|jmavsim"`** —— 任何跳出组的孤儿进程
   按名收(纯 grep 模式匹配,jMAVSim 的进程名匹配就杀;不影响其它软件)

实现见 `scripts/gs_replay_random.sh` 的 `start_sitl` / `stop_sitl`。

### 3.3 命名空间一致性

录制和回放都用 `uav_name:=uav0`(默认)启动,保持 `starling_bridge` 在 `/uav0/*` 下发布。
换 namespace(例如 `uav1`)时需要:

1. 给 `gs_record.launch` 加 `starling_pub_topics:=<新清单>`
2. 给 `gs_play.launch` 加 `uav_name:=uav1`
3. 给 `gs_replay_random.sh` 加 `--state-topic=/uav1/mavros/state --uav-name=uav1`

---

## 4. CLI 一览

### `gs_record.launch`

| arg | 默认 | 含义 |
|---|---|---|
| `uav_name` | `uav0` | UAV namespace,需要和 starling_bridge 列表对齐 |
| `flow` | `ground_search_track` | 暂未自启 gs_simulator,此 arg 保留兼容 |
| `enable_rviz` | `true` | RViz 开关 |
| `record_bag` | `true` | 是否启 rosbag record |
| `bag_dir` | `~/LOG` | bag 落盘目录 |
| `bag_name` | `gs_test_play_recorded` | bag 文件名(不带 .bag) |
| `starling_pub_topics` | 见 §2.1 | 18 个白名单话题 |

### `gs_play.launch`

| arg | 默认 | 含义 |
|---|---|---|
| `uav_name` | `uav0` | 同上 |
| `enable_rviz` | `true` | |
| `bag_dir` | `~/LOG` | |
| `bag_path` | `~/LOG/gs_test_play_recorded.bag` | 待回放的 bag 绝对路径 |
| `play_rate` | `1.0` | 回放倍率 |
| `record_playback_bag` | `true` | 是否同步全录 playback bag |
| `playback_bag_name` | `gs_playback_recorded` | 落盘文件名(不带 .bag) |

### `gs_replay_random.sh`

| flag | 默认 | 含义 |
|---|---|---|
| `--bag-dir DIR` | `~/LOG` | 输入 bag 检索根 |
| `--bag-prefix PREFIX` | `gs_test_play_` | |
| `--loop N` | `1` | 跑几只 |
| `--rate R` | `1.0` | 回放倍率 |
| `--startup-delay S` | `12` | roslaunch 后 Mavros 多等秒数 |
| `--post-delay S` | `5` | bag 走完再多等秒数再 SIGINT |
| `--uav-name NAME` | `uav0` | UAV namespace |
| `--start-sitl` / `--no-start-sitl` | start | 是否脚本内启 SITL |
| `--sitl-script PATH` | `~/src/PX4_Firmware/Tools/sitl_single_run.sh` | |
| `--sitl-wait S` | `22` | SITL 启动后等多秒再进 loop |
| `--record-playback-bag` / `--no-record-playback-bag` | on | 是否同步全录 playback bag |
| `--playback-bag-name NAME` | `gs_playback_recorded` | |
| `--state-topic TOPIC` | `/uav0/mavros/state` | analyze 阶段扫的话题 |
| `--auto-land-stop` / `--no-auto-land-stop` | stop | FAIL 是否立即停 loop |
| `--report-file PATH` | `/tmp/gs_replay_report.txt` | analyze 输出落盘路径 |
| `--dry-run` | off | 只打印配置,不动 SITL / 不跑 roslaunch |

---

## 5. 已知限制 & 下一版待加

1. **录制/回放都没做 startup delay**
   - 录制:手动 Ctrl+C 之后开始录,绕开 Mavros 没起的问题
   - 回放:bag 前 10s 的 GS 命令大概率被 Mavros 没订阅完吞掉。
   - 下一版可给 `gs_play.launch` 加 `<arg name="startup_delay">` 配合 `launch-prefix="bash -c 'sleep N && exec "$@"'"` 来延迟播放单独一步。
2. **AUTO.LAND 是粗糙探针**
   - 实际"出 land"原因很多:看 `mission/detect_targets` 是否提前清空、`mavros/state.armed` 何时变 false、`/failsafe/position` 是否触发,这里只看 mode 一项。
   - 下一版在 `/tmp/gs_replay_report.txt` 里补"为什么 land"段,把失败前 5s 的关键话题 dump 出来。
3. **analyze 只看 `mavros/state`**
   - 没看 `mission/detect_targets`、`/failsafe/*`、uav_avoidance 的 output。
   - 扩展方式:在 `analyze_playback_bag` 里加 `topics=topics.split(",")` 列表,通用扫 + 模式匹配即可。
4. **evaluate 打分**
   - 当前只检 land,没有"任务完没完成"打分。
   - 有了 playback bag,可在 analyze 阶段后跑一个独立的 `analyze_*` 脚本,根据 flow 类型任务流对到达距离 / Attack 时机 / 完成时间综合打分。
5. **每轮 replay 用同一只 playback bag 名时,会互相覆盖**
   - 想每次留档用 `--playback-bag-name=gs_playback_$(date +%s)`。
6. **starling_bridge 在 play 阶段仍占着 KCP 端口**
   - 没有真 GS,KCP socket 是空的,所以不会自扰;若以后做 fail-safe 压力测试,加 `--exclude-node starling_bridge`。
