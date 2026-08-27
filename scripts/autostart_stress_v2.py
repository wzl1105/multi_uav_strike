#!/usr/bin/env python3
"""
multi_uav_strike 自启动可靠性压测 v2
====================================
在 v1 基础上加了三个独立维度的健康检查:

  维度 A: 节点存活 (rosnode ping -c 1) ── 检测 "注册了但崩了" 的伪存活节点
  维度 B: systemd Restart 计数 (NRestarts 增量) ── 检测 crash-loop
  维度 C: 关键 topic 频率采样 (rostopic hz) ── 检测 "活着但不工作"

注意: 板子 IP 不再硬编码, 通过 --host 参数传入; 不再假定特定 IP.

用法 (PC 上):
  python3 autostart_stress_v2.py --host 192.168.1.152 --password rockchip
  python3 autostart_stress_v2.py --host 192.168.1.152 --password rockchip --max-cycles 50
  python3 autostart_stress_v2.py --host 192.168.1.152 --password rockchip --no-reboot  # 只跑 1 轮不重启
  python3 autostart_stress_v2.py --host 192.168.1.152 --password rockchip --no-check-topics  # 跳过维度 C topic 频率检测
"""

import subprocess, time, json, os, argparse, sys
from datetime import datetime
from collections import OrderedDict

# ==================== 默认配置 (CLI 可覆盖) ====================
DEFAULT_USER = "root"
DEFAULT_PASSWORD = "rockchip"  # 默认值, 推荐通过 --password 传
ROS_DISTRO = "noetic"
WORKSPACE_SETUP = "/home/rpdzkj/tzs_ws/devel/setup.bash"

# 从 single_uav_px4_flight_test.launch + ruiyan_sdk.launch 提取的期望节点
EXPECTED_NODES = [
    "/rosout",
    "/ruiyan_ros_node",
    "/uav0/mavros",
    "/target_motion_simulator_node",
    "/uav0/target_estimator_node",
    "/uav0/guidance_control_node",
    "/uav0/waypoint_executor_node",
    "/uav0/mission_manager_node",
    "/uav0/uav_avoidance_node",
    "/uav0/starling_bridge",
    "/uav0/power_interact_node",
    "/starling_gimbal",
    "/flight_rosbag_record",
]

# 维度 C: 关键 topic 频率采样. (node, topic, min_hz, max_hz, sample_sec)
# max 给一些宽容, 实际允许到 2x max 算 ok
TOPIC_HZ_PROBES = [
    ("/uav0/mavros",         "/uav0/mavros/state",                       0.5,  3.0, 8),
    ("/uav0/guidance_control_node", "/uav0/cmd/stop",                   0.0, 50.0, 5),  # 该命令可能没有, ok 只是允许 0
]

STABILIZATION_TIMEOUT = 60      # 1 分钟内全部齐
POLL_INTERVAL = 4               # 4 秒一次
REBOOT_WAIT = 25
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "autostart_test_logs")
# =============================================================


def which(prog):
    from shutil import which as _which
    return _which(prog)


def make_ssh(host, password):
    """构造 sshpass 基础 args"""
    return [
        "sshpass", "-p", password,
        "ssh", "-o", "StrictHostKeyChecking=no",
        "-o", "ConnectTimeout=5",
        "-o", "UserKnownHostsFile=/dev/null",
        f"{DEFAULT_USER}@{host}",
    ]


def ssh_run(host, password, cmd, timeout=10):
    ssh_cmd = make_ssh(host, password) + [cmd]
    try:
        r = subprocess.run(ssh_cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout, r.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "ssh-timeout"


def get_nodes(host, password):
    """
    rosnode list. 必须显式 source ROS env.
    关键:
      - bash -c (不是 -lc): 避免 login shell 多 2-3 秒
      - 末尾 < /dev/null: 关掉 stdin, 否则 bash 等 stdin 关才返回
    """
    rc, out, _ = ssh_run(
        host, password,
        f"bash -c 'source /opt/ros/{ROS_DISTRO}/setup.bash >/dev/null 2>&1; "
        f"source {WORKSPACE_SETUP} >/dev/null 2>&1; rosnode list' < /dev/null",
        timeout=8,
    )
    if rc != 0:
        return []
    return [n.strip() for n in out.splitlines() if n.strip().startswith("/")]


def ping_all_nodes(host, password, nodes):
    """
    维度 A: rosnode ping -c 1 对每个节点. 一次 SSH 完成所有节点.
    返回 {node: bool}
    注: rosnode ping 没有 --timeout 选项 (默认 3s), 加了会 rc=2 全 dead
    """
    if not nodes:
        return {}
    quoted = " ".join(f'"{n}"' for n in nodes)
    cmd = (
        f"bash -c 'source /opt/ros/{ROS_DISTRO}/setup.bash >/dev/null 2>&1;"
        f"source {WORKSPACE_SETUP} >/dev/null 2>&1;"
        f"for n in {quoted}; do"
        f" rosnode ping \"$n\" -c 1 >/dev/null 2>&1 && echo \"$n:alive\" || echo \"$n:dead\";"
        f"done' < /dev/null"
    )
    # 每个 ping 最多等 3s, 加 SSH 开销
    rc, out, _ = ssh_run(host, password, cmd, timeout=8 + len(nodes) * 3)
    results = {}
    for line in out.splitlines():
        if ":" in line:
            node, status = line.rsplit(":", 1)
            results[node.strip()] = status.strip() == "alive"
    return results


def get_restart_counts(host, password):
    """
    维度 B: systemctl show ... -p NRestarts --value
    返回 {service_name: int}
    """
    rc, out, _ = ssh_run(
        host, password,
        "systemctl show ruiyan.service tzs_ws-multi.service -p NRestarts --value 2>&1",
        timeout=5,
    )
    parts = out.split()
    return {
        "ruiyan.service":      int(parts[0]) if len(parts) >= 1 and parts[0].lstrip("-").isdigit() else 0,
        "tzs_ws-multi.service": int(parts[1]) if len(parts) >= 2 and parts[1].lstrip("-").isdigit() else 0,
    }


def probe_topic_hz(host, password, topic, sample_sec=8):
    """
    维度 C: rostopic hz 跑 sample_sec, 取 average rate.
    返回实际 Hz (float). 失败返回 0.0.
    """
    cmd = (
        f"bash -c 'source /opt/ros/{ROS_DISTRO}/setup.bash >/dev/null 2>&1;"
        f"source {WORKSPACE_SETUP} >/dev/null 2>&1;"
        f"timeout {sample_sec - 1} rostopic hz {topic} 2>&1' < /dev/null | tail -5"
    )
    rc, out, _ = ssh_run(host, password, cmd, timeout=sample_sec + 3)
    actual = 0.0
    for line in out.splitlines():
        if "average rate:" in line:
            try:
                actual = float(line.split()[-1])
            except (ValueError, IndexError):
                pass
    return actual


def get_service_state(host, password):
    rc, out, _ = ssh_run(
        host, password,
        "systemctl is-active ruiyan.service tzs_ws-multi.service 2>&1",
        timeout=8,
    )
    return out.strip() if rc == 0 else ""


def get_launch_log_tail(host, password):
    rc, out, _ = ssh_run(
        host, password,
        "tail -n 80 /var/log/tzs_ws/multi_uav_strike.log 2>/dev/null",
        timeout=8,
    )
    return out if rc == 0 else ""


def get_image_version(host, password):
    """
    读 TZS_IMAGE_VERSION, 失败返回 None.
    注: start_gimbal.sh 的 log() 走 stdout -> journalctl, 不进 roslaunch log 文件
    """
    rc, out, _ = ssh_run(
        host, password,
        "journalctl -u tzs_ws-multi.service --since '10 minutes ago' 2>/dev/null | "
        "grep -oE 'TZS_IMAGE_VERSION=[0-9]+\\.[0-9]+\\.[0-9]+' | tail -1",
        timeout=8,
    )
    if rc == 0 and "=" in out:
        return out.split("=", 1)[1].strip()
    return None


def reboot_board(host, password):
    print(f"[{datetime.now():%H:%M:%S}] 重启 {host} ...")
    ssh_cmd = [
        "sshpass", "-p", password, "ssh",
        "-o", "StrictHostKeyChecking=no",
        "-o", "UserKnownHostsFile=/dev/null",
        f"{DEFAULT_USER}@{host}", "sudo reboot",
    ]
    subprocess.run(ssh_cmd, capture_output=True, timeout=5)
    time.sleep(REBOOT_WAIT)
    for _ in range(40):
        rc, _, _ = ssh_run(host, password, "echo alive", timeout=5)
        if rc == 0:
            print(f"[{datetime.now():%H:%M:%S}] {host} 已上线")
            return True
        time.sleep(2)
    return False


def run_cycle(cycle_id, host, password, prev_restart, log_f, no_reboot, check_topics=True):
    cycle_start = time.time()
    seen = OrderedDict()
    expected = set(EXPECTED_NODES)
    log_f.write(f"\n--- 第 {cycle_id} 轮 @ {datetime.now()} | host={host} ---\n")
    log_f.flush()
    print(f"\n=== 第 {cycle_id} 轮 开始（{STABILIZATION_TIMEOUT}s 内齐） ===")

    image_version = get_image_version(host, password)
    if image_version:
        print(f"  映像版本: TZS_IMAGE_VERSION={image_version}")

    success = False
    extra = []
    while True:
        elapsed = time.time() - cycle_start
        if elapsed > STABILIZATION_TIMEOUT:
            break
        nodes = get_nodes(host, password)
        for n in nodes:
            if n not in seen:
                if n in expected:
                    seen[n] = elapsed
                    line = f"  [{elapsed:5.1f}s] + {n}"
                    print(line)
                    log_f.write(line + "\n")
                else:
                    extra.append((n, elapsed))
        if not (expected - set(seen.keys())):
            success = True
            break
        time.sleep(POLL_INTERVAL)

    # 维度 A: 节点存活
    health = {}
    if success:
        print("  [A] 节点存活 (rosnode ping):")
        health = ping_all_nodes(host, password, EXPECTED_NODES)
        for n in EXPECTED_NODES:
            alive = health.get(n, False)
            mark = "✓" if alive else "✗"
            line = f"    {mark} {n}"
            print(line)
            log_f.write(line + "\n")

    # 维度 B: Restart 计数
    restart = get_restart_counts(host, password)
    restart_diff = {}
    for k, v in restart.items():
        restart_diff[k] = v - prev_restart.get(k, 0)
    print("  [B] systemd Restart:")
    for k, v in restart.items():
        diff = restart_diff[k]
        suffix = f" (+{diff} 本轮)" if diff > 0 else ""
        line = f"    {k}: {v}{suffix}"
        print(line); log_f.write(line + "\n")
        if diff > 0:
            warn = f"    ⚠ {k} 本轮重启了 {diff} 次 (crash-loop?)"
            print(warn); log_f.write(warn + "\n")

    # 维度 C: 关键 topic 频率 (可选跳过, --no-check-topics)
    hz_results = []
    if success and check_topics:
        print("  [C] 关键 topic 频率 (rostopic hz):")
        for node, topic, mn, mx, sample in TOPIC_HZ_PROBES:
            if node in EXPECTED_NODES and not health.get(node, True):
                # 节点死了, 跳过
                line = f"    - {topic}: 节点 {node} dead, 跳过"
                print(line); log_f.write(line + "\n")
                hz_results.append({"topic": topic, "actual_hz": None, "skipped": True})
                continue
            actual = probe_topic_hz(host, password, topic, sample)
            ok = (mn <= actual <= max(mx * 2, mn + 1)) if mn == 0 else (mn <= actual <= mx * 2)
            mark = "✓" if ok else "✗"
            line = f"    {mark} {topic}: {actual:.2f} Hz (期望 {mn}-{mx} Hz)"
            print(line); log_f.write(line + "\n")
            hz_results.append({"topic": topic, "actual_hz": actual, "expected_min": mn, "expected_max": mx, "ok": ok})
    elif not check_topics:
        print("  [C] 关键 topic 频率: SKIPPED (--no-check-topics)")

    missing = sorted(expected - set(seen.keys()))
    dead = [n for n, alive in health.items() if not alive]
    summary = {
        "cycle": cycle_id,
        "ts": datetime.now().isoformat(),
        "host": host,
        "image_version": image_version,
        "status": "success" if success and not dead else "failure",
        "elapsed_s": round(time.time() - cycle_start, 2),
        "expected_first_seen_s": seen,
        "unexpected_nodes": [{"node": n, "first_seen_s": t} for n, t in extra],
        "missing": missing,
        "dead_nodes": dead,
        "ping_health": health,
        "restart_counts": restart,
        "restart_diff": restart_diff,
        "hz_results": hz_results,
        "service_state": get_service_state(host, password),
    }
    json_path = os.path.join(LOG_DIR, f"cycle_{cycle_id:04d}.json")
    with open(json_path, "w") as jf:
        json.dump(summary, jf, indent=2, ensure_ascii=False)

    if success and not dead:
        msg = f"  全部 {len(expected)} 节点齐 + 全部 alive, 耗时 {summary['elapsed_s']}s ✓"
        print(msg); log_f.write(msg + "\n")
    else:
        msg = (f"  ✗ 失败 (missing={missing}, dead={dead}) "
               f"service 状态: {summary['service_state']}")
        print(msg); log_f.write(msg + "\n")
        tail = get_launch_log_tail(host, password)
        if tail:
            log_f.write("\n=== multi_uav_strike.log tail ===\n" + tail + "\n")
    log_f.flush()

    return (success and not dead), summary, restart


def main():
    ap = argparse.ArgumentParser(
        description="multi_uav_strike 自启动可靠性压测 v2 (含 ping / Restart / Hz 三维度)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--host", required=True, help="目标板 IP (如 192.168.1.152)")
    ap.add_argument("--password", default=DEFAULT_PASSWORD,
                    help=f"SSH 密码 (默认 {DEFAULT_PASSWORD})")
    ap.add_argument("--max-cycles", type=int, default=10**6, help="最大循环次数")
    ap.add_argument("--no-reboot", action="store_true",
                    help="跑 1 轮后停止, 不重启板子 (调试用)")
    ap.add_argument("--no-check-topics", action="store_true",
                    help="跳过维度 C 关键 topic 频率采样 (rostopic hz), "
                         "只跑节点存活 + Restart 计数。默认要检测 topic。")
    args = ap.parse_args()

    if which("sshpass") is None:
        sys.exit("ERROR: 需要安装 sshpass: sudo apt install sshpass")

    # 预检
    rc, _, _ = ssh_run(args.host, args.password, "echo PING_OK", timeout=5)
    if rc != 0:
        sys.exit(f"ERROR: SSH 连不上 root@{args.host}, 检查 IP/密码/网络")

    os.makedirs(LOG_DIR, exist_ok=True)
    main_log = os.path.join(LOG_DIR, "test_run.log")

    with open(main_log, "a") as log_f:
        log_f.write(f"\n========== 测试开始 v2 {datetime.now()} | host={args.host} ==========\n")
        cycle = 0
        prev_restart = {"ruiyan.service": 0, "tzs_ws-multi.service": 0}
        try:
            while cycle < args.max_cycles:
                cycle += 1
                ok, summary, restart = run_cycle(
                    cycle, args.host, args.password, prev_restart, log_f,
                    no_reboot=args.no_reboot,
                    check_topics=not args.no_check_topics,
                )
                prev_restart = restart
                if not ok:
                    stop = (f"\n>>> 第 {cycle} 轮失败, 按要求【不再重启】, 留现场。\n"
                            f">>> SSH 进板子查:\n"
                            f">>>   journalctl -u tzs_ws-multi.service -n 200\n"
                            f">>>   /var/log/tzs_ws/multi_uav_strike.log\n"
                            f">>>   systemctl status ruiyan.service tzs_ws-multi.service\n"
                            f">>> 详细时序: {LOG_DIR}/cycle_{cycle:04d}.json\n")
                    print(stop); log_f.write(stop)
                    break
                if args.no_reboot:
                    break
                if not reboot_board(args.host, args.password):
                    log_f.write(">>> 重启失败, 停止\n")
                    break
        except KeyboardInterrupt:
            print("\n中断"); log_f.write("\n中断\n")
    print(f"\n日志: {LOG_DIR}")


if __name__ == "__main__":
    main()