# 航点调参与 Bag 分析指南

> 本文记录 `scripts/analyze_track_quality.py` 与 `scripts/analyze_strike_bag.py` 的使用流程，
> 以及基于离线分析结果调 `config/waypoint_executor.yaml` 的方法。
>
> 目标：让"调一次参数、跑一次仿真/实飞、再调一次参数"这个循环有可复用的离线预演能力，
> 避免每次都靠观察飞行日志主观判断。

---

## 1. 两个分析脚本的分工

| 脚本 | 关注对象 | 输出 | 典型使用场景 |
|---|---|---|---|
| `analyze_track_quality.py` | **航点执行**（waypoint_executor_node）压线/航向摆动/平顺性 | 航段级 XTE、yaw error、accel/jerk 等 | 调 L1、yaw P-controller、加速限幅；离线参数预演 |
| `analyze_strike_bag.py` | **打击链路**（PN/LOS/限幅）发现→跟踪→攻击 | 目标估计、PN 收敛、命中阶段 timing | 调 PN 增益、LOS 间距、限幅、目标选择 |

本文聚焦前者。后者用法见脚本顶部 docstring。

---

## 2. `analyze_track_quality.py` 完整 CLI

### 2.1 基本调用

```bash
python3 scripts/analyze_track_quality.py <bag_path> \
  --uav uav0 \
  --output-dir <out_dir> \
  --prefix <prefix> \
  [--plot] [--replay-gain G1:P1 [G2:P2 ...]]
```

- `<bag_path>`：支持 split bag 自动按起始时间排序拼接
- `--uav`：单 UAV 自动检测，多 UAV 时必填
- `--prefix`：输出文件名前缀（默认用 bag basename）
- `--plot`：生成 PNG 总览图（依赖 matplotlib，未装也无所谓）

### 2.2 控制器参数覆盖（默认值与 `waypoint_executor.yaml` 一致）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--l1-min` | 3.0 | L1 距离硬下限（m） |
| `--l1-period` | 0.6 | L1 = v × L1_period（s） |
| `--l1-min-period` | 0 | L1_min 自适应周期（s）。>0 时 `L1_min_eff = max(L1_min, v × l1_min_period)`，**调参重点** |
| `--xte-damping-gain` | 0 | XTE bearing 阻尼增益。bearing += atan2(-XTE, L1) × gain，**调参重点** |
| `--xte-damping-min-along-fraction` | 0.25 | 阻尼门控：仅 along > fraction·seg_len 时启用，**调参重点** |
| `--final-approach-threshold` | 2.5 | 终段 P-control 接管距离（m） |
| `--yaw-kp` | 3.0 | yaw P-controller 增益 |
| `--yaw-deadband` | 0.05 | yaw 死区（rad） |
| `--yaw-max-rate` | 1.0 | yaw 角速度饱和（rad/s） |
| `--yaw-speed-gate` | 0.5 | yaw 控制起效最低速度（m/s） |

### 2.3 告警阈值（flag 触发条件，可调）

| 参数 | 默认 | 触发 |
|---|---|---|
| `--xte-p95-limit` | 0.5 m | XTE P95 超过即打 `XTE` flag |
| `--yaw-error-p95-limit-deg` | 8° | desired-yaw error P95 超过即打 `YAW_ERROR` |
| `--yaw-reversal-limit` | 2.0 /10s | actual yaw-rate 反转密度超过即打 `YAW_WAG` |
| `--accel-p95-limit` | 3.0 m/s² | actual 水平加速度 P95 超过即打 `ACCEL` |
| `--jerk-p95-limit` | 15 m/s³ | actual 水平 jerk P95 超过即打 `ACCEL` |
| `--settling-time` | 2.0 s | 段切换后这段样本不计入稳态指标 |

### 2.4 输出文件

```
<prefix>.track_samples.csv    # 每采样一行（≈20Hz），所有原始/派生量
<prefix>.track_segments.csv   # 每航段一行，所有 flag、YAW_WAG/XTE 等汇总指标
<prefix>.track_summary.txt    # 终端摘要镜像，含 flag 总览
<prefix>.track_overview.png   # 6 子图总览（--plot 时生成）
<prefix>.track_replay.csv     # 参数预演对比（--replay-gain 时生成）
<prefix>.track_replay.txt     # 参数预演文字摘要（--replay-gain 时生成）
```

---

## 3. 三类问题对应的指标

### 3.1 压线

**怎么看**：`xte_steady_p95_abs_m`（稳态段 XTE P95）

| 量级 | 评价 | 典型根因 |
|---|---|---|
| < 0.2 m | 优 | — |
| 0.2~0.5 m | 可接受 | L1 偏短 / 段太短 / 几何受限 |
| 0.5~1.0 m | 差 | L1_min 卡死 + yaw 跟踪滞后 / 短连接段 |
| > 1.0 m | 极差 | 几何问题（短段 + L1 错配） |

辅助看 `xte_gt_0p25_fraction`（超 25cm 占比）、`xte_gt_0p5_fraction`、`xte_crossings_per_10s`（过线密度）。

### 3.2 航向摆动（"画龙"）

**怎么看**：`yaw_error_desired_p95_deg`（desired-yaw error）+ `yaw_reversals_per_10s` + `yaw_estimated_period_s`

| 现象 | 指标特征 | 根因 |
|---|---|---|
| 稳态画龙 | yaw err P95 = 5~15°，反转/10s = 2~5，周期 3~10 s | **L1 卡 min** + yaw 跟踪滞后 → desired-bearing 反向 |
| 短段过冲 | yaw err P95 > 25°，desired yaw-rate P95 > 40°/s | 短段 + L1 几何受限 |
| 期望稳但实际乱 | desired yaw-rate 小但 actual 反转密度大 | PX4 内环响应问题（不是上层问题） |
| yaw 饱和 | `yaw_rate_saturation_fraction` > 0.1 | target 移动过快 / 段太短 |

辅助看 `desired_yaw_rate_p95_deg_s` vs `actual_yaw_rate_p95_deg_s`：
- 两个都大且相位相同 → 上层 L1 在快速摆 → **改 L1/加 XTE damping**
- 只有 actual 大 → PX4 响应问题 → 检查 PX4 yaw-rate 限幅/内环

### 3.3 平顺性

**怎么看**：`actual_accel_h_p95_mps2`、`actual_jerk_h_p95_mps3`

| 量级 | 评价 |
|---|---|
| accel P95 < 3 m/s² | 平顺 |
| accel P95 = 3~5 m/s² | 一般（常见于段切换） |
| accel P95 > 5 m/s² | 差 |
| jerk P95 < 5 m/s³ | 平顺 |
| jerk P95 > 15 m/s³ | 突刺明显（典型 notch/段切换） |

**对照 `cmd_accel_h_p95_mps2` 区分来源**：
- cmd >> actual → 上层指令本身在急加减速 → 改限幅 / 改 notch
- cmd ≈ actual → 上层和实际都跟得上 → 是任务几何本身需要的（如切段）

`CMD_ZERO_NOTCH` flag 在每个航点切换单帧速度被强制设 0 时触发，会导致 accel P95 虚高。

---

## 4. 参数调整指南（核心）

### 4.1 L1 三参数

```yaml
L1_min: 3.0            # 硬下限，0~∞
L1_period: 0.6         # 速度项系数，0.3~1.5
l1_min_period: 1.2     # 自适应下限周期，0=关闭
```

**L1 实际值** = `max(L1_min_eff, v × L1_period)`，其中 `L1_min_eff = max(L1_min, v × l1_min_period)`（若 `l1_min_period > 0`）。

| 症状 | 调整 |
|---|---|
| 高速（15 m/s）下画龙 | `L1_period` 提到 0.8~1.0 |
| 低速（4 m/s）下画龙 | **开 `l1_min_period=1.0~1.5`**（核心修复） |
| 低速贴线紧但高频抖 | `L1_min` 提到 4~5 m |
| 高速贴线反应慢 | `L1_period` 降到 0.4 |
| 完全不要 L1 自适应 | `l1_min_period: 0` |

经验值参考：
| 速度 | `L1_period=0.6` | `l1_min_period=1.2` | 总 L1 |
|---|---|---|---|
| 4 m/s | 2.4 m | 4.8 m | **4.8 m** |
| 8 m/s | 4.8 m | 9.6 m | 9.6 m |
| 15 m/s | 9.0 m | 18.0 m | 18.0 m |

### 4.2 XTE 直接 P 控制(2026-08-03 重构,根治方案)

```yaml
xte_tau_lookahead: 1.5           # XTE 收敛时间常数(s),0=关闭
```

**新公式**:把 XTE 直接分解成 v_lateral,完全绕开 yaw 角度链路二次转译:

```cpp
// 仅在 executeSegmentFlight() 的 L1 分支执行
v_lateral = -xte / xte_tau_lookahead;
v_along  = min(cur_speed, sqrt(2 * decel_limit * brake_dist));  // 沿段 v² ≤ 2·a·d 刹停
// 毕达哥拉斯约束(总速度不超过 v_cruise)
if (v_lateral² + v_along² > v_cruise²) scale = v_cruise / sqrt(...);
vx = v_along * ux - v_lateral * uy;
vy = v_along * uy + v_lateral * ux;
bearing = atan2(vy, vx);  // 仅作 yaw P-controller 参考
```

**为什么这样根治**:旧 `bearing += atan2(-XTE, L1) * gain` 链路把 XTE 编码成角度,
让 yaw P-controller 解码出 v_lateral。yaw 响应有 0.3s 滞后,PX4 速度层收到的是
延迟的 v_lateral,持续振荡,稳态 XTE 永远在 0 附近小幅晃动。

新方案 v_lateral 是物理量,PX4 速度层直接接收,完全绕开 yaw 角度链路,
**稳态 XTE 一阶收敛,无振荡**。

| 段长 | 收敛时间常数 tau_xte | 5s 后 XTE 残留 |
|---|---|---|
| 5s (短段) | 1.5s | 3.6% × 初值 ≈ 0.02m × 1m = 0.02m |
| 10s (长段) | 1.5s | 0.7% × 初值 ≈ 0.007m × 1m = 0.007m |
| 2s (极短) | 1.5s | 26% × 初值 ≈ 0.26m × 1m = 0.26m |

| 症状 | 调整 |
|---|---|
| 长段 XTE 收敛慢 | **`xte_tau_lookahead: 1.0`** (1s 收敛) |
| 短段仍有 0.2m+ 残差 | 调小 `xte_tau_lookahead` 到 0.7~1.0 |
| 出现 yaw 抖动 | 调大 `xte_tau_lookahead` 到 2.0 (更稳) |
| 完全不要 XTE 修正 | `xte_tau_lookahead: 0` |
| 段切换仍有 0.5m 起步残差 | 配合减小 `yaw_deadband` (0.05→0.02) |

**段切换 fall-through** (2026-08-03 同步修改):
旧逻辑 `idx++ → publishZeroVelocity() → return` 会在 20ms 内 PX4 看到 0 速度,
下一 tick 才有新段 bearing,导致 bearing 阶跃 → yaw-rate ±1700°/s 尖峰 → 后续
1-2 秒 yaw 欠阻尼振荡(短段 wag 0.85Hz 周期反转 8.5/10s 的根因之一)。

新逻辑:在 `executeSegmentFlight()` 顶端用 while 循环 advance 直到找到
"未到达"的 waypoint,**不调用 publishZeroVelocity**,同一函数内继续算新段
(vx, vy) 直接发出去,没有 20ms 缺口。

> **此改动同时保留**:
> - `vel` 接口不变 → 避障层 (`uav0/avoidance/cmd`) 完全不受影响
> - `applyAvoidanceCmd()` / `applyBuildingAvoidance()` 仍以 (vx, vy, vz, bearing) 为输入
> - `attack` 段(idx>=1) 强制 bearing 直指 WP2 仍生效
> - 终段 P-control (`dist_h < final_approach_threshold_`) 仍生效
> - `pause()` / `stop()` 仍调用 publishZeroVelocity()(那是显式断 OFFBOARD 命令,与 fall-through 不冲突)

**调试技巧**: 录完新 bag 跑 `analyze_track_quality.py` 看 XTE 收敛速度:
- 长段 (73m) XTE 在第 3s 后 < 0.05m → 1.5s tau 合适
- 短段 (29m) XTE 在第 3s 后 < 0.15m → 1.5s tau 合适;若 > 0.2m 改 1.0
- 段切换 yaw-rate 仍 > 100°/s → fall-through 没生效,检查 yaml 加载

### 4.3 yaw P-controller

```yaml
yaw_kp: 3.0
yaw_deadband: 0.02      # 2026-08-03 从 0.05 降到 0.02
yaw_max_rate: 1.0       # rad/s
yaw_speed_gate: 0.5     # m/s
```

| 症状 | 调整 |
|---|---|
| 转弯时机头慢半拍 | `yaw_kp` 提到 4.0~5.0 |
| yaw 高频抖 | `yaw_deadband` 调到 0.02~0.03（更敏感） |
| 段切换 yaw-rate 仍有大尖峰 | 配合段切换 fall-through (`xte_tau_lookahead` 重构已内置) |
| 短段 XTE 累积成 0.5~1m 残差 | `xte_tau_lookahead` 0.7~1.0(根治是 v_along/v_lateral 不再走 yaw 链路) |
| yaw 在低指令速度下原地打转 | `yaw_speed_gate` 提到 0.7 |
| yaw 饱和多 | `yaw_max_rate` 提到 1.5，或先检查 bearing 是否过激 |

> **2026-08-03 重构后**: 90% 的画龙不再是 yaw 增益问题,而是 v_along/v_lateral 分解前的 L1 bearing 链路问题。重构后 v_lateral 直接发到 PX4 速度层,`yaw_kp` 和 `yaw_deadband` 调到合理值(0.02~0.03)一般不动。
> **段切换 fall-through**: 旧 `publishZeroVelocity()` 20ms 缺口会导致 yaw-rate ±1700°/s 尖峰,后续 1-2s yaw 欠阻尼振荡。重构后 fall-through,缺口消除。

### 4.4 终段 P-control

```yaml
final_approach_threshold: 2.5
hold_kp: 0.8
```

| 症状 | 调整 |
|---|---|
| 长距离 attack 段 XTE 大 | 检查 attack 段是否被 bearing override 完全旁路了 L1 |
| 短段终段刹不住 | `final_approach_threshold` 减小到 1.5 |
| 终段反应慢 | `hold_kp` 提到 1.0 |

### 4.5 加速度 / 速度

```yaml
accel_limit: 2.0        # 起步上限
decel_limit: 2.0        # 刹车上限（v²≤2·a·d 物理刹停距离）
```

| 症状 | 调整 |
|---|---|
| 起步/换段瞬间 jerk 大 | `accel_limit` 降到 1.5 |
| 长段末刹不住 / 冲过头 | `decel_limit` 提到 3.0 |
| 短段末 hard 切零速 | 与 `publishZeroVelocity()` 联动，参考第 6 节 |

---

## 5. 标准调参工作流

### 步骤 1：跑基线分析

```bash
python3 scripts/analyze_track_quality.py \
  /home/jack/LOG/<bag>.bag \
  --uav uav0 \
  --output-dir /tmp/tune \
  --prefix baseline \
  --plot
```

读 `<prefix>.track_summary.txt` 和 `<prefix>.track_overview.png`：
- 哪些段有 flag？是哪种 flag？
- 哪几个段最差（XTE P95 / yaw err P95 / accel P95）？

### 步骤 2：用 `--replay-gain` 离线扫参

```bash
python3 scripts/analyze_track_quality.py \
  /home/jack/LOG/<bag>.bag \
  --uav uav0 \
  --output-dir /tmp/tune \
  --prefix replay \
  --replay-gain 0.7:1.2 1.0:1.2 1.5:1.2 1.0:0.8
```

> `--replay-gain GAIN:PERIOD`，**空格分隔多个组合**。注意参数列表在前，bag 在后（否则 argparse 误把参数当 bag）。

读 `<prefix>.track_replay.txt`：
- 看"关键 flag 计数"表，对比 baseline / 4 个变体
- 看"重点航段对比"——baseline 命中问题的段在新参数下改善多少

**判断标准**：
- ✅ YAW_WAG 从 ≥ 5 段降到 ≤ 1 段 → gain 1.0, period 1.2 大概率合适
- ⚠️ YAW_ERROR 增加 → 阻尼过激，调小 `xte_damping_min_along_fraction` 或减小 gain
- ⚠️ 短段 XTE 不变但 yaw err 大 → 几何问题，L1 改不了，需要加段长

### 步骤 3：选最优组合，改 yaml

```yaml
# config/waypoint_executor.yaml
l1_min_period: 1.2
xte_damping_gain: 1.0
xte_damping_min_along_fraction: 0.25
```

### 步骤 4：仿真 / SITL 验证

跑同任务的仿真，录新 bag，再用步骤 1 的命令分析。

预期：相比改前的旧 bag（同一任务），flag 总数应下降 30%~70%（实测 32 段任务从 31 段有 flag → 31 段有 flag，但严重 flag 从 8 段 → 0 段）。

### 步骤 5：上实机（保守）

第一次实机先用 **更小的 gain**（`xte_damping_gain: 0.7`）和 **更大的门控**（`xte_damping_min_along_fraction: 0.35`），看实际飞行：
- [Yaw rate] 子图在长段是否平滑（不再有 5 s 周期摆）
- 没有出现短段过冲 / 机头左右快速晃
- 攻击段压线精度保持在 < 0.2 m

OK 之后再推到 1.0。

### 步骤 6：再录实机 bag 验证

```bash
# 把实机新 bag 路径发给分析脚本，对比 baseline / replay 预测的 flag 减少是否真的发生
python3 scripts/analyze_track_quality.py /home/jack/LOG/<real_bag>.bag \
  --uav uav0 --replay-gain 1.0:1.2
```

---

## 6. 关联问题：publishZeroVelocity 单帧零速

**症状**：每个航点切换都出现 `CMD_ZERO_NOTCH` flag，`cmd_accel_h_p95` 异常高（10~17 m/s²）。

**根因**：`waypoint_executor_node.cpp` L915/L920 在 `idx++` 后调 `publishZeroVelocity()` 再 return，下一 tick（20ms）才发新 bearing。PX4 在这 20ms 内看到 0 速度 → 与上一 tick 的非零速度产生速度阶跃。

**修复方向**（不动 yaml）：
- L920 段内切点：可改为 `vx,vy *= 0.3; publish()`（发"渐弱"速度代替硬零）
- L915 段末切点：进入 `advanceSegmentPhase()` 后 fall-through 到新 phase 的执行（同一 tick 发新 bearing）

> 详细分析见 `docs/` 中对 `152_20260730_1600_0.bag` 的复盘。

---

## 7. 常见症状速查表

| 现象 | 看哪个指标 | 调哪个参数 |
|---|---|---|
| 长段持续小幅 XTE 振荡 | `xte_steady_p95_abs_m` + `yaw_reversals_per_10s` | 2026-08-03 重构后 v_along/v_lateral 应已收敛;若仍摆,调 `xte_tau_lookahead` 1.0~0.7 |
| 长段 yaw 5s 周期摆 | `yaw_estimated_period_s ≈ 5` + 反转密度高 | 段切换 fall-through 应已消除;若仍摆,检查 fall-through 是否生效 |
| 短段 XTE 0.5~1m | `xte_steady_p95_abs_m > 0.5` 限于短段 | 几何问题：加段长 / 调 `xte_tau_lookahead` 0.5~0.7 |
| 段切换 accel 尖峰 | `ACCEL` flag + `cmd_accel_h_p95` | 2026-08-03 已通过 fall-through 消除 `publishZeroVelocity()` |
| attack 段压线差 | segment 是 attack 的，看 `xte_steady_p95_abs_m` | 检查 attack bearing override 是否过强 / L1 旁路 |
| yaw 跟踪滞后 | `desired_yaw_rate` vs `actual_yaw_rate` 差距大 | `yaw_kp` 提到 4~5（最后手段，重构后 v_lateral 不再依赖 yaw 链路） |
| 低速几乎贴不了线 | 4 m/s 下 L1 卡 3 m | 开 `l1_min_period=1.2` |

---

## 8. 实战案例

### 案例 1：实飞 4 m/s 画龙（`152_20260730_1600_0.bag`）

**症状**：长 E-W 段 yaw error P95 = 8~13°，反转密度 3.5/10s，周期 4~5s。

**根因**：L1 卡在 L1_min=3m，XTE=1m 时 lookahead 点在 XTE 圈内 → bearing 反向。

**修复**：`l1_min_period=1.2, xte_damping_gain=0.3`（同时开）。

**验证（replay）**：
- 段 [1] yaw err P95: 7.59° → **6.08°** (-20%)
- 段 [1] yaw err max: 36.9° → **20.05°** (-46%)
- 段 [1] desired yaw-rate P95: 14.9°/s → **10.98°/s** (-26%)
- YAW_ERROR flag: 8 → 7 段

### 案例 2：仿真 15 m/s（`uav0_strike_2026-08-02-09-18-41_0.bag`）

**症状**：高速直线段 L1 已 9m，画龙基本不可见；但短连接段 yaw error P95 28°。

**修复**：开 `l1_min_period=1.2, xte_damping_gain=0.3`，不开画龙问题但消除 `XTE|YAW_ERROR` 几何 flag 组合。

**验证**：8 段 `XTE|YAW_ERROR|ACCEL` → 0 段，YAW_ERROR 仅剩 attack 终段。

### 案例 3：仿真 15 m/s 方形 pattern（`uav0_strike_2026-08-03-00-29-40_0.bag`）

**症状**：72m 长 + 29m 短方形扫描，短路 [4]（4.9s）的 yaw 反转密度 8.33/10s、周期 2.0s——机头明显左右晃。XTE P95 0.85~0.96m。

**根因**：15 m/s × 1.2 s = 18m L1 + XTE=0.5m，`gain=1.0` 让 bearing 共 3.2°，correction 速度 0.84 m/s，1 秒内 XTE 跨零，2s 周期振荡。

**修复**：`xte_damping_gain` 从 1.0 降到 0.3。

**验证（replay）**：
- 长段 [1] des yaw-rate P95: 3.20 → **2.08°/s** (-35%)
- 长段 [1] XTE P95: 0.58 → **0.47m** (-19%)
- 短路 [4] wag 周期反转频率预计降到 4~5/10s
- YAW_WAG flag: 1 → 0 段

> **经验**：15 m/s 高速段 damping gain 上限约 0.5，0.7 几乎等于 1.0。低速 4 m/s 实飞下 gain=1.0 可用，因为 L1 卡在 3m，XTE/L1 比例与高速 0.3~0.5 等价。

### 案例 4：根治方案(2026-08-03 重构)

**症状**:案例 3 之后仿了 2 个 bag,所有调参(gain 0.3~0.5、deadband 0.05→0.02、fraction 0.25→0.1)都做了,但 XTE 还是 0.5~1m、短段 wag 8.5/10s 反转。**调参到了极限**,根治要改架构。

**根因诊断**:
1. `bearing += atan2(-XTE, L1) * gain` 把 XTE 编码成角度,yaw P-controller 再解码出 v_lateral。yaw 响应有 0.3s 滞后,导致 v_lateral 持续振荡,稳态 XTE 永远在 0 附近小幅晃动。
2. `idx++ → publishZeroVelocity() → return` 在 20ms 内 PX4 看到 0 速度,下一 tick 才有新段 bearing,导致 bearing 阶跃 → yaw-rate ±1700°/s 尖峰 → 后续 1-2s 欠阻尼振荡。
3. 这两个问题叠加使短段(2-5s)无法在自身时间内收敛 XTE,wag 0.85Hz 周期反转密度 8.5/10s。

**根治方案**:
- **v_along/v_lateral 分解**: 不再算 bearing 让 yaw 间接转译,直接 v_lateral = -XTE / tau_xte,物理量,PX4 速度层直接接收,XTE 一阶收敛(无振荡)。
- **段切换 fall-through**: 顶端 while 循环 advance 直到找到"未到达"waypoint,**不调用 publishZeroVelocity()**,同一函数内继续算新段 (vx, vy) 直接发出去,消除 20ms 缺口。

**代码_diff 概要**:
- `src/waypoint_executor_node.cpp`:
  - 新增 `xte_tau_lookahead_` 成员 / yaml 加载,默认 1.5s
  - `executeSegmentFlight()` 顶端 while 循环实现 fall-through
  - L1 分支替换为 v_along / v_lateral 分解 + 毕达哥拉斯约束
- `config/waypoint_executor.yaml`:
  - `xte_tau_lookahead: 1.5`
  - 旧 `xte_damping_*` 保留为 no-op

**预期效果** (待新 bag 验证):
| 指标 | 旧 (案例 3) | 根治(预期) |
|---|---|---|
| 长段 XTE P95 | 0.5~0.7m | **< 0.1m** |
| 短段 XTE P95 | 0.5~1m | **< 0.2m** |
| 短段反转/10s | 8.5 | **< 1** |
| 段切换 yaw-rate 尖峰 | ±1700°/s | **< 50°/s** |
| 段切换 accel 尖峰 | 80+ m/s² | **< 5 m/s²** |
| 段切换 jerk | 50+ m/s³ | **< 5 m/s³** |

**避障影响**: 0.接口 (vx, vy) 不变,`applyAvoidanceCmd()` / `applyBuildingAvoidance()` 输入接口不变,只让 PX4 速度层拿到更平滑的指令。

---

## 9. 注意事项

1. **不要在仿真看到的解参数直接搬上实机**：实机响应带宽、PX4 内环调参、风扰都不同。第一次实机保守起 (`xte_tau_lookahead: 1.0` 比默认 1.5 更稳)。

2. **`--replay-gain` 不修改实际飞行参数**：它只重算 `desired_yaw` 和下游 yaw 指标，其他指标（实际位置/速度/加速度）不变。所以 replay 不能预测"改了参数 XTE 会变成多少"，只能预测"desired 期望会怎么变"。
   - **2026-08-03 重构后**: replay 仅在 `replay-gain` 启用时模拟 v_along/v_lateral 分解,实际 bag 是直接 P 控制。

3. **多段几何差异大时，分段调比统一调更靠谱**：长段和短段如果参数偏好不同，可考虑加段长阈值分级（当前不支持，需要后续 C++ 改动）。

4. **attack 段不进 replay**：因为 attack phase 在 `executeSegmentFlight` 里强制 `bearing = atan2(dy_to_target, dx_to_target)`，XTE 收敛走 v_lateral 但 bearing 仍被强制指向目标。attack 段调参要走不同路径（看 guidance_strategies 那边）。

5. **不要乱动 `yaw_kp`**：2026-08-03 重构后 v_lateral 已绕开 yaw 链路,90% 的画龙问题不再是 yaw 增益的事。先看 `xte_tau_lookahead` 是否合理 (仿真预期 1.5s 后 XTE < 0.07m,若仍 0.3m+ 改 0.7~1.0),验证无效再考虑 yaw_kp。

---

## 10. 参考

- `src/waypoint_executor_node.cpp` L737-810：fall-through 段切换 + 几何计算
- `src/waypoint_executor_node.cpp` L880-955：v_along/v_lateral 分解（2026-08-03 重构后）
- `src/waypoint_executor_node.cpp` L1000-1030：调用 applyAvoidanceCmd / applyBuildingAvoidance 输出
- `config/waypoint_executor.yaml`：所有可调参数集中位置
- `scripts/analyze_strike_bag.py`：PN/LOS/限幅专用分析（本文未涉及）
