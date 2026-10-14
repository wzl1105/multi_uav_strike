# Changelog

Versions follow `MAJOR.MINOR.PATCH` (semver-ish, not strict semver):
- `PATCH` (0.0.x): bug fix / small functional change
- `MINOR` (0.x.0): significant new feature
- `MAJOR` (x.0.0): breaking change to behavior, public interface, or onboard protocol

The version number is stored in `package.xml` `<version>...</version>` and is
read by `deploy_windows.bat query <ip>`.

The format below is intentionally minimal — one line per version, no headers.

## 0.0.1 (2026-10-10, baseline on nopic_nostrike)
- Initial versioned baseline on `nopic_nostrike` (commit de15815 "控制接管").
- Scope of this baseline (only what is actually on this branch as of de15815):
  manual recognition RC + topic entry, control take-back on override,
  manual identify switch, 30m no-image no-strike setpoint continuity.
- Tooling: `tools/update_tools/{deploy_linux.sh, deploy_windows.bat, backup_to_local.sh,
  grab_bag.sh, grab_bag.bat}`. `deploy_windows.bat query <ip>` to inspect.
- NOTE: target_estimator iterations (v16/v17/v18/v76/v78/v79/v80 bearing EKF
  etc.) live on the `refactor/classifier-solver-split` branch, NOT on
  `nopic_nostrike`. They are NOT part of 0.0.1.

## 0.0.2 (on nopic_nostrike, 2026-10-10)

相对 0.0.1 的变化。

### 新功能
- 手动拉烟触发: RC PWM (低边沿, hysteresis, armed-only) + `/yx/cmd/trigger_smoke`
  Bool topic (边沿触发), 0.2s 防抖。跟 strike_test_smoke 自动路径独立。
- `LN_config.{sh,bat}` 菜单加 `[G] 拉最新 bag`、`[T] 手动拉烟`、`[Q] 查板子版本`。
  之前的 deploy 脚本只能 deploy。
- `LN_config.bat` 整段脚本 SSH 密码**只问 1 次** (旧版问 5+ 次; SSH_ASKPASS
  模式绕开 Windows OpenSSH ControlMaster bug,旧版在 Windows 上静默失败)。

### 用户能感知的 bug 修复
- `/trigger_smoke` 在 500ms YX burst 内不再被 reject (`allow_interrupt=true`)。
- 拉烟 service 瞬时失败不再 latch + 终止整次 strike: 改成 3x200ms 重试,
  3 次都失败才 latch。
- 拉烟 0.2s 速度/位置稳定门槛删了: 位置锁存即触发。"有时不触发"根因之一。
- Attack 选 primary 改用 YOLO back-projection 优先: 修 "YOLO 断流期间 stale
  cluster 被 fresh 错 cluster 抢占" 的 bug。
- 飞手接管期间 GCS 不再收到 `detect_target` / `detect_targets`, 松开自动恢复。
- flight_test launch 收紧版 mavros plugin 白名单加 `manual_control`: RC override
  在 POSCTL/MANUAL 模式下可达 FCU (旧版静默丢)。
- `LN_config T` (trigger smoke) hostname drift 修: 板子端 `ROS_HOSTNAME=localhost`,
  旧版 `$(hostname)` 解析到 `/etc/hosts` stale IP, echo subscriber 永远连不上
  → T 操作在这块板子上等于无效。
- 打击后不再向地面站发送新目标，根据地面站新任务判断是否需要回传目标。

### 工具改名
- `tools/update_tools/deploy_linux.sh` → `LN_config.sh`,
  `deploy_windows.bat` → `LN_config.bat`。旧名作为同签名 alias 仍可用。

## 0.0.3 (on nopic_nostrike, 2026-10-12)

相对 0.0.2 的变化。
摇杆量69 70消息测试成功
打击状态维持原状不变

## 0.0.4 (on nopic_nostrike, 2026-10-12)

相对 0.0.3 的变化。
chase 末段 body yaw 修符号 (NWU→NED bearing 翻号),bbox 不再出 FOV
橙色大球 (target_est_marker) 移出 silence 闸,STRIKE 全程可见

## 0.0.5 (on nopic_nostrike, 2026-10-13)

相对 0.0.4 的变化。
chase 阶段 yaw 控制指令反向,进入 chase 后不再 180° 反向转向
(09-42-52 bag, `guidance_control_node.cpp:2627` `atan2(ray.y,ray.x)` → `-atan2(...)`,
抵消 `currentYawNed()` 里的 `-yaw_nwu` 负号)

## 0.0.6 (on nopic_nostrike, 2026-10-14)

相对 0.0.5 的变化。
chase 阶段取消latch，选择目标点最近的活目标，降低误识别率
修复搜索跟踪内部的latch限制
打击的高度保护在10m范围内再考虑4m