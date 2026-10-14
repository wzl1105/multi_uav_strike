#!/usr/bin/env python3
"""
分析 multi_uav_flight_21_0.bag 是否进了搜索跟踪模式 + 飞机实际是否在跟踪目标:
  - phase 时序 (4 = PHASE_GUIDANCE_TRACK)
  - work_mode 时序 (4 = SEARCH_TRACK)
  - tracking_state.state (0/1)
  - UAV 位置 vs primary_image_target / cluster / target_position
  - attack_cmd_received
  - PX4 mode
"""
import os
import sys
import struct
import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main(bag_path):
    name = os.path.basename(bag_path).replace('.bag', '')
    print(f'\n=== {name} ===')

    bag = rosbag.Bag(bag_path, 'r')

    # 1) mission_state
    ms_t, ms_phase, ms_attack_cmd, ms_skill, ms_strike_committed = [], [], [], [], []
    for _, msg, t in bag.read_messages(topics=['/uav0/mission/mission_state']):
        ms_t.append(t.to_sec())
        ms_phase.append(msg.phase)
        ms_attack_cmd.append(msg.attack_cmd_received)
        ms_strike_committed.append(msg.strike_committed)
        ms_skill.append(msg.skill_id)
    ms_t = np.array(ms_t)
    ms_phase = np.array(ms_phase, dtype=int)
    ms_attack_cmd = np.array(ms_attack_cmd, dtype=int)
    ms_strike_committed = np.array(ms_strike_committed, dtype=int)
    ms_skill = np.array(ms_skill)

    # 2) work_mode (mission_manager -> GS)
    wm_t, wm = [], []
    for _, msg, t in bag.read_messages(topics=['/uav0/mission/work_mode']):
        wm_t.append(t.to_sec())
        wm.append(msg.mode)
    wm_t = np.array(wm_t)
    wm = np.array(wm, dtype=int)

    # 3) tracking_state
    ts_t, ts_state, ts_dist, ts_lat, ts_lon = [], [], [], [], []
    for _, msg, t in bag.read_messages(topics=['/uav0/mission/tracking_state']):
        ts_t.append(t.to_sec())
        ts_state.append(msg.state)
        ts_dist.append(msg.target_dist)
        ts_lat.append(msg.target_lat)
        ts_lon.append(msg.target_lon)
    ts_t = np.array(ts_t)
    ts_state = np.array(ts_state, dtype=int)
    ts_dist = np.array(ts_dist, dtype=float)

    # 4) PX4 mode
    sm_t, sm_mode, sm_armed = [], [], []
    for _, msg, t in bag.read_messages(topics=['/uav0/mavros/state']):
        sm_t.append(t.to_sec())
        sm_mode.append(msg.mode)
        sm_armed.append(msg.armed)
    sm_t = np.array(sm_t)
    sm_mode = np.array(sm_mode)
    sm_armed = np.array(sm_armed, dtype=int)
    # mode: 0=MANUAL 1=ACRO 3=AUTO/STABILIZED 4=ACRO 5=OFFBOARD ... 实际 PX4 mode 字符串
    # 简单把 0/非0 当作 "OFFBOARD vs 其它"
    sm_is_offboard = (sm_mode == 'OFFBOARD')

    # 5) UAV 本地位置
    uav_t, uav_x, uav_y, uav_z = [], [], [], []
    for _, msg, t in bag.read_messages(topics=['/uav0/mavros/local_position/pose']):
        uav_t.append(t.to_sec())
        p = msg.pose.position
        uav_x.append(p.x); uav_y.append(p.y); uav_z.append(p.z)
    uav_t = np.array(uav_t)
    uav_x = np.array(uav_x); uav_y = np.array(uav_y); uav_z = np.array(uav_z)

    # 6) primary_image_target — 字段: center_x/y, world_ray_nwu (单位向量)
    pit_t, pit_visible, pit_cid, pit_label = [], [], [], []
    for _, msg, t in bag.read_messages(topics=['/uav0/guidance/primary_image_target']):
        pit_t.append(t.to_sec())
        pit_visible.append(int(msg.visible))
        pit_cid.append(int(msg.cluster_id))
        pit_label.append(str(msg.label))
    pit_t = np.array(pit_t)
    pit_visible = np.array(pit_visible)

    # 7) target_position (target_estimator 输出, target 位置)
    tp_t, tp_x, tp_y, tp_z = [], [], [], []
    for _, msg, t in bag.read_messages(topics=['/target_position']):
        tp_t.append(t.to_sec())
        tp_x.append(msg.x); tp_y.append(msg.y); tp_z.append(msg.z)
    tp_t = np.array(tp_t)
    tp_x = np.array(tp_x); tp_y = np.array(tp_y); tp_z = np.array(tp_z)

    bag.close()

    # ===== 报告 =====
    print(f'\n--- 频率 ---')
    print(f'  mission_state: {len(ms_t)} samples @ {len(ms_t)/max(1,(ms_t[-1]-ms_t[0])):.2f} Hz')
    print(f'  work_mode:     {len(wm_t)} samples (mode 改动次数)')
    print(f'  tracking_state:{len(ts_t)} samples @ {len(ts_t)/max(1,(ts_t[-1]-ts_t[0])):.2f} Hz')
    print(f'  UAV pose:      {len(uav_t)} samples')
    print(f'  primary_image_target: {len(pit_t)} samples')
    print(f'  target_position: {len(tp_t)} samples')

    if len(ms_t) == 0:
        print('  WARN: no mission_state')
        return

    # 唯一 phase 集合
    uniq = np.unique(ms_phase, return_counts=True)
    phase_names = {0:'GROUND_IDLE',1:'TAKING_OFF',2:'HOVERING',3:'WAYPOINT_FOLLOW',
                   4:'GUIDANCE_TRACK',5:'GUIDANCE_STRIKE',6:'RETURNING',7:'COMPLETE',
                   8:'FAILED',9:'HOLDING',10:'OVERRIDDEN',11:'TARGET_LOST_HOVER'}
    print(f'\n--- phase 分布 ---')
    for p, c in zip(uniq[0], uniq[1]):
        nm = phase_names.get(int(p), f'?({p})')
        dur = c * np.median(np.diff(ms_t)) if len(ms_t) > 1 else 0
        print(f'  phase={p:>2} {nm:>20} : {c:>4} 次  (~{dur:.1f}s)')

    # GUIDANCE_TRACK (phase=4) 段
    track_mask = ms_phase == 4
    track_t = ms_t[track_mask]
    print(f'\n--- PHASE_GUIDANCE_TRACK (phase=4) 段 ---')
    if len(track_t) > 0:
        # 连续段
        dts = np.diff(track_t)
        seg_starts = [track_t[0]]
        seg_ends = []
        for i, dt in enumerate(dts):
            if dt > 0.5:
                seg_ends.append(track_t[i])
                seg_starts.append(track_t[i+1])
        seg_ends.append(track_t[-1])
        print(f'  共 {len(seg_starts)} 个连续段:')
        for s, e in zip(seg_starts, seg_ends):
            print(f'    [{s-ms_t[0]:6.2f}s, {e-ms_t[0]:6.2f}s]  持续 {e-s:.2f}s')
    else:
        print('  ❌ 从未进 phase=4 PHASE_GUIDANCE_TRACK')

    # work_mode 序列
    print(f'\n--- work_mode 序列 (5=SEARCH_STRIKE 4=SEARCH_TRACK) ---')
    if len(wm_t) > 0:
        for i in range(len(wm_t)):
            t_rel = wm_t[i] - ms_t[0]
            print(f'  t={t_rel:6.2f}s  work_mode={wm[i]} ({"SEARCH_TRACK" if wm[i]==4 else "SEARCH_STRIKE" if wm[i]==5 else "?"})')
    else:
        print('  no work_mode messages')

    # tracking_state.state 序列 (0=未跟踪 1=跟踪中)
    print(f'\n--- tracking_state.state 持续时间 ---')
    if len(ts_t) > 0:
        uniq_ts = np.unique(ts_state, return_counts=True)
        for s, c in zip(uniq_ts[0], uniq_ts[1]):
            label = '跟踪中' if s == 1 else '未跟踪'
            dur = c * np.median(np.diff(ts_t)) if len(ts_t) > 1 else 0
            print(f'  state={s} ({label}): {c} samples  ~{dur:.1f}s')
        # 第一次切到 1 的时刻
        first_track = np.where(ts_state == 1)[0]
        if len(first_track) > 0:
            t0 = ts_t[first_track[0]] - ms_t[0]
            print(f'  第一次切到"跟踪中"时刻: t={t0:.2f}s')
        # 看 UAV 在 tracking 期间的"距离"变化
        if 'ts_dist' in dir() and len(ts_dist) > 0:
            track_idx = ts_state == 1
            if track_idx.sum() > 0:
                print(f'  tracking 期间 target_dist: 起始={ts_dist[track_idx][0]:.1f}m  末尾={ts_dist[track_idx][-1]:.1f}m  min={ts_dist[track_idx].min():.1f}m  max={ts_dist[track_idx].max():.1f}m')
    else:
        print('  no tracking_state')

    # attack_cmd
    print(f'\n--- attack_cmd_received ---')
    print(f'  收到次数 = {ms_attack_cmd.sum()} (max={ms_attack_cmd.max()})')
    if ms_attack_cmd.sum() > 0:
        ac_idx = np.where(ms_attack_cmd > 0)[0]
        for i in ac_idx:
            print(f'    t={ms_t[i]-ms_t[0]:6.2f}s  skill_id={ms_skill[i]}')

    # strike_committed
    print(f'\n--- strike_committed ---')
    print(f'  总触发次数 = {ms_strike_committed.sum()}')

    # PX4 mode 时序 (统计 OFFBOARD 时间)
    print(f'\n--- PX4 mode ---')
    if len(sm_t) > 0:
        uniq_mode = np.unique(sm_mode, return_counts=True)
        for m, c in zip(uniq_mode[0], uniq_mode[1]):
            print(f'  mode={m}: {c} samples')
        off_t = sm_t[sm_mode == 'OFFBOARD']
        nonoff_t = sm_t[sm_mode != 'OFFBOARD']
        if len(off_t) > 0:
            print(f'  OFFBOARD 总时长: {off_t[-1]-off_t[0]:.1f}s ({len(off_t)} samples)')

    # ===== 跟踪行为: UAV 位置 vs primary_image_target / target_position 距离 =====
    print(f'\n--- 跟踪行为: UAV 离目标距离 (NWU 平面) ---')
    if len(uav_t) > 0 and len(tp_t) > 0:
        # UAV 位置插到 tp_t
        uav_x_on_tp = np.interp(tp_t, uav_t, uav_x)
        uav_y_on_tp = np.interp(tp_t, uav_t, uav_y)
        d_2d = np.sqrt((uav_x_on_tp - tp_x)**2 + (uav_y_on_tp - tp_y)**2)
        d_3d = np.sqrt((uav_x_on_tp - tp_x)**2 + (uav_y_on_tp - tp_y)**2 + (uav_z.mean() - tp_z)**2)
        print(f'  target_position 距离 (NWU 平面): start={d_2d[0]:.1f}m  end={d_2d[-1]:.1f}m  min={d_2d.min():.1f}m  max={d_2d.max():.1f}m')

    # ===== 画图 =====
    t0 = ms_t[0]
    ms_t_rel = ms_t - t0
    if len(uav_t): uav_t_rel = uav_t - t0
    if len(pit_t): pit_t_rel = pit_t - t0
    if len(tp_t): tp_t_rel = tp_t - t0
    if len(ts_t): ts_t_rel = ts_t - t0
    if len(sm_t): sm_t_rel = sm_t - t0

    fig, axes = plt.subplots(5, 1, figsize=(14, 14), sharex=True)

    axes[0].plot(ms_t_rel, ms_phase, 'b-', lw=1.2, label='mission_state.phase')
    axes[0].set_yticks(list(phase_names.keys()))
    axes[0].set_yticklabels([phase_names[k] for k in phase_names.keys()], fontsize=7)
    axes[0].set_ylabel('phase')
    axes[0].axhline(4, color='g', ls='--', lw=0.5, alpha=0.5, label='TRACK=4')
    axes[0].axhline(5, color='r', ls='--', lw=0.5, alpha=0.5, label='STRIKE=5')
    axes[0].set_title(f'{name}: 是否进搜索跟踪?')
    axes[0].legend(loc='upper right', fontsize=7)
    axes[0].grid(True, alpha=0.3)

    if len(sm_t):
        axes[1].plot(sm_t_rel, sm_is_offboard.astype(int), 'm-', lw=1, label='PX4 OFFBOARD')
        axes[1].set_ylabel('mode bool')
        axes[1].set_yticks([0, 1])
        axes[1].set_yticklabels(['非 OFFBOARD', 'OFFBOARD'], fontsize=8)
        axes[1].legend(loc='upper right', fontsize=7)
        axes[1].grid(True, alpha=0.3)

    if len(ts_t):
        axes[2].plot(ts_t_rel, ts_state, 'g-', lw=1, label='tracking_state.state')
        axes[2].set_ylabel('tracking')
        axes[2].set_yticks([0, 1])
        axes[2].set_yticklabels(['未跟踪', '跟踪中'], fontsize=8)
        axes[2].legend(loc='upper right', fontsize=7)
        axes[2].grid(True, alpha=0.3)

    if len(tp_t) and len(uav_t):
        axes[3].plot(tp_t_rel, tp_x, 'r-', lw=0.6, label='target.x (NWU)')
        axes[3].plot(tp_t_rel, tp_y, 'b-', lw=0.6, label='target.y (NWU)')
        axes[3].plot(uav_t_rel, uav_x, 'r--', lw=0.5, alpha=0.5, label='UAV.x')
        axes[3].plot(uav_t_rel, uav_y, 'b--', lw=0.5, alpha=0.5, label='UAV.y')
        axes[3].set_ylabel('pos (m)')
        axes[3].legend(loc='upper right', fontsize=7)
        axes[3].grid(True, alpha=0.3)

    if len(tp_t) and len(uav_t):
        uav_x_on_tp = np.interp(tp_t, uav_t, uav_x)
        uav_y_on_tp = np.interp(tp_t, uav_t, uav_y)
        d_2d = np.sqrt((uav_x_on_tp - tp_x)**2 + (uav_y_on_tp - tp_y)**2)
        axes[4].plot(tp_t_rel, d_2d, 'k-', lw=1, label='UAV→target 平面距离 (m)')
        axes[4].set_ylabel('dist (m)')
        axes[4].set_xlabel('time (s, from mission_state start)')
        axes[4].legend(loc='upper right', fontsize=7)
        axes[4].grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(os.path.dirname(bag_path), f'{name}_track_phase.png')
    plt.savefig(out, dpi=80)
    plt.close()
    print(f'\n  -> {out}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('usage: analyze_track_phase.py <bag>')
        sys.exit(1)
    main(sys.argv[1])
