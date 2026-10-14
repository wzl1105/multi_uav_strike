#!/usr/bin/env python3
"""
从 bag 抓 /uav0/rosout, 过滤 mission_manager / work_mode / phase / SEARCH_TRACK / GUIDANCE 相关日志,
按时间顺序输出, 帮找 "为什么 phase=4 PHASE_GUIDANCE_TRACK 没进"。
"""
import os
import sys
import rosbag


def main(bag_path):
    name = os.path.basename(bag_path).replace('.bag', '')
    print(f'\n=== {name} rosout dump (mission_manager / phase / SEARCH_TRACK / work_mode) ===\n')

    bag = rosbag.Bag(bag_path, 'r')
    rows = []
    for _, msg, t in bag.read_messages(topics=['/uav0/rosout', '/rosout', '/rosout_agg']):
        try:
            m = msg.msg
        except AttributeError:
            m = msg
        try:
            txt = m
        except Exception:
            txt = str(msg)
        # rosgraph_msgs/Log: msg, level, name, file, line, function, topics
        try:
            node = msg.name
            lvl = msg.level
            fn = msg.function
        except Exception:
            continue
        keep = False
        keys = ['MissionManager', 'mission_manager', 'PHASE_', 'work_mode',
                'SEARCH_TRACK', 'SEARCH_STRIKE', 'GUIDANCE', 'TaskFlow',
                'workmode', 'phase', 'task_flow', 'skill', 'PX4 mode',
                'OFFBOARD', 'POSCTL', 'override', 'OVERRIDDEN', 'promote',
                'REJECT', 'reject', 'blocked', 'failed', 'guidance', 'cluster']
        for k in keys:
            if k in txt:
                keep = True
                break
        if not keep:
            continue
        rows.append((t.to_sec(), node, fn, lvl, txt))

    bag.close()
    rows.sort(key=lambda r: r[0])

    if not rows:
        print('  (no matching rosout)')
        return

    t0 = rows[0][0]
    for ts, node, fn, lvl, txt in rows:
        rel = ts - t0
        # 简化: 去掉前缀空行 / 多余空白
        txt_one = ' '.join(txt.split())
        print(f'  t={rel:7.2f}s  [{node}/{fn}]  {txt_one}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('usage: dump_rosout_phase.py <bag>')
        sys.exit(1)
    main(sys.argv[1])
