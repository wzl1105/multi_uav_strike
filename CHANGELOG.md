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
- Smoke trigger priority fix: `/trigger_smoke` service now allows interrupting
  in-flight YX command sequences (`allow_interrupt=true`). Root cause for
  "smoke not firing sometimes": the 0x69/0x87/0x4E/0xD2 bursts are 10 frames
  @ 50ms = 500ms, and a second `/trigger_smoke` call within 500ms was rejected
  with `allow_interrupt=false`. Smoke is an end-of-task emergency action;
  guarantee its execution. Aligned with `/yx/cmd` (cmd=4) which already
  used `allow_interrupt=true`. Touches `yx_comm_node.cpp:419-431`.
- Smoke trigger retry: both auto (`maybeTriggerStrikeTestSmoke`) and manual
  (`tryManualSmokeTrigger`) paths now retry up to 3 times at 200ms interval
  on service failure. Old behavior: lock `request_attempted_=true` and give
  up the entire strike on first failure. Root cause for "auto smoke never
  fires on first try" — a 0x69 (stage2_release) tail-end collision lasted
  ~500ms, smoke call was rejected, latch was set, no recovery for the rest
  of the strike. New behavior: first failure does NOT lock the latch, retry
  3x200ms; only after all 3 fail is the latch set. Service-unavailable is
  treated as permanent (lock immediately) since retrying won't help.
- Manual smoke debounce: 1.0s -> 0.2s minimum interval between calls.
  Aligns with "instant feedback" intent for the manual override path.
- Manual smoke trigger via two new entry points (independent of existing
  strike_test_smoke auto path):
  - `mavros/rc/in` channel with on/off PWM hysteresis (default off, low-side
    active). Mirrors `manual_recognition_rc_*` style.
  - `std_msgs/Bool` on `/yx/cmd/trigger_smoke`, edge-triggered latch.
- Safety gate: armed-only. Disarm on ground -> both paths REJECT with WARN.
  1.0s minimum interval between calls to debounce RC PWM jitter / topic re-pub.
- New params (guidance_control.yaml): `manual_smoke_rc_enabled`,
  `manual_smoke_rc_channel`, `manual_smoke_rc_on_pwm`,
  `manual_smoke_rc_off_pwm`, `manual_smoke_rc_timeout_sec`,
  `manual_smoke_topic_enabled`, `manual_smoke_topic`.
- New tools: `tools/update_tools/grab_bag.{sh,bat}` for on-site engineers
  to stop rosbag and SCP the latest .bag to local without power-cycling.
- `deploy_windows.bat query <ip>` subcommand to inspect version on the board.
- 飞手接管后停发 detect_target / detect_targets: 105 Attack 流程在执行中
  且飞手把 OFFBOARD 切走时,mission_manager 不再往 GCS 上行定位信息。
  接管 = 任务终止,接管后 UAV 不受机载控制,绿球位置不可信,
  type=1 (strike trigger / strike commit) 也无意义。飞手松开回 OFFBOARD
  立即恢复,无需新 TaskFlow。
- 拉烟触发条件简化: 位置锁存 (高度 ≤4m 硬保护 或 bbox hover 持续 N 帧)
  即触发拉烟,不再等 0.2s 速度/位置稳定。原"速度未达阈值永远不触发"
  是"有时不触发"主因之一。删除 5 个 yaml 参数和 7 个 cpp 字段。
