#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gs_simulator_node.py
地面站模拟器节点(只支持 typed TaskFlow,不再发 /gs/* legacy topics)

使用方法:
  $ rosrun multi_uav_strike gs_simulator_node.py _flow:=ground_search_strike _uav_ns:=uav0

可选 flow:
- ground_search_strike / ground_search_track / ground_gather_search / catapult_search_return / ground_search
- b47c_normal / groupb1bc  (真实任务数据,内置 ref_lat/lon 覆盖)

若无 flow 参数(空字符串),节点不发布任何东西,纯哑节点,等待外部 GS 推 TaskFlow。
"""

import rospy
import geometry_msgs.msg
import mavros_msgs.msg
import multi_uav_strike.msg as mus_msg
import sys
import argparse
import time
import json
from math import cos, pi


# ============================================================================
# Phase 2+: 内嵌 DEMO TaskFlows(每套对应一个测试场景)
# ============================================================================
# 坐标系:相对参考点的米(NED 北, NED 东, 上为正高度)
# skill_type (当前): 100=Takeoff-弹射, 101=Gather, 102=Search, 103=Return, 105=Attack, 106=Takeoff-地面
#
# 起飞按类型已拆分:
#   100 = Takeoff-弹射 (catapult takeoff, 走 mission/catapult_trigger 流程)
#   106 = Takeoff-地面 (ground takeoff, 走原 OFFBOARD+ARM 流程)
# 所有 demo 已按此分好,弹射起飞 demo 仍带 takeoff_subtype=1 标记(后续弃用),
# 地面起飞 demo 带 takeoff_subtype=0 (后续弃用)。

DEMO_FLOWS = {
    # ---------- Demo 1: 地面起飞 + 搜索 + 打击 ----------
    # 坐标系: NED (北=+x, 东=+y, 上=+z, 米)
    # 目标位置 NED: (-30, 10, 0)  ← 由 target_motion_simulator.yaml (x_init=-30, y_init=10) 决定
    # 相机方向: pitch=-90°(正下视), FOV 60°,
    #   在高度 z=30m 时地面可视半径 ≈ 30*tan(30°) ≈ 17m,正下视能看到目标
    # 搜索策略: 在 x=-30 这条南北线上飞,从 y=-30 飞到 y=50,会掠过目标 y=10
    "ground_search_strike": {
        "flow_id": "demo_ground_search_strike_001",
        "work_mode": 5,  # SEARCH_STRIKE — 识别即上报并自动打击
        "skills": [
            # 1) Takeoff (地面起飞, takeoff_subtype=0)
            {
                "skill_id": "takeoff_001",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,  # 相对 PX4 home,目标 alt = home_alt + 30
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],  # 起飞不需 arrive_path
                "skill_area_path": [],
            },
            # 2) Search(到达搜索区,识别目标)
            {
                "skill_id": "search_001",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 6.0,
                "arrive_path": [
                    (0.0, -30.0, 30.0),   # 原地起飞到 30m 后先飞往搜索线南端上方
                ],
                "skill_area_path": [
                    (-30.0, -30.0, 30.0),  # 搜索线南端 (目标在 y=10,这里在下边)
                    (-30.0, 10.0, 30.0),   # 正对目标 (y=10 与目标 x/y 一致) — 应该在这里看到目标
                    (-30.0, 20.0, 30.0),   # 越过目标继续搜索
                ],
            },
            # 3) Attack(WP1=发现点,WP2=瞄准点,task_speed 是打击速度)
            # 目标点约在 (-30, 10, 0),瞄准点下降到 5m
            {
                "skill_id": "attack_001",
                "skill_type": 105,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 12.0,
                "arrive_path": [],
                "skill_area_path": [
                    (-30.0, 40.0, 30.0),  # WP1 = 发现点(目标上空)
                    (-30.0, 60.0, 0.0),   # WP2 = 瞄准点(下降到低空)
                ],
            },
            # 4) Return(返航)
            {
                "skill_id": "return_001",
                "skill_type": 103,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 0.0, 30.0),  # 返航点(原点上空)
                ],
            },
        ],
    },

    # ---------- Demo 2: 地面起飞 + 搜索跟踪(等地面站确认) ----------
    "ground_search_track": {
        "flow_id": "demo_ground_search_track_001",
        "work_mode": 4,  # SEARCH_TRACK — 识别后锁定 + 等地面站 attack_cmd 确认
        "skills": [
            {
                "skill_id": "takeoff_001",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,  # 相对 PX4 home,目标 alt = home_alt + 30
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                "skill_id": "search_track_001",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 6.0,
                "arrive_path": [
                    (50.0, 0.0, 30.0),
                    (50.0, 50.0, 30.0),
                ],
                "skill_area_path": [
                    (50.0, 50.0, 30.0),
                    (50.0, 150.0, 30.0),
                ],
            },
            {
                "skill_id": "return_001",
                "skill_type": 103,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 0.0, 30.0),
                ],
            },
        ],
    },

    # ---------- Demo 3: 地面起飞 + 集结等待(单机模拟等待另一 UAV) ----------
    "ground_gather_search": {
        "flow_id": "demo_ground_gather_search_001",
        "work_mode": 3,  # SEARCH_ONLY — 纯搜索,只上报不锁不打
        "skills": [
            {
                "skill_id": "takeoff_001",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,  # 相对 PX4 home,目标 alt = home_alt + 30
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                "skill_id": "gather_001",
                "skill_type": 101,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 4.0,  # 集结不需任务速度
                "arrive_path": [
                    (20.0, 0.0, 30.0),
                ],
                "skill_area_path": [],  # 集结只需到 ARRIVE 点,无独立 skill_area (mission_manager 检测到空路径会跳过 ENTRY_PENDING)
            },
            {
                "skill_id": "search_001",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 6.0,
                "arrive_path": [
                    (50.0, 0.0, 30.0),
                ],
                "skill_area_path": [
                    (50.0, 50.0, 30.0),
                    (50.0, 150.0, 30.0),
                ],
            },
            {
                "skill_id": "return_001",
                "skill_type": 103,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 0.0, 30.0),
                ],
            },
        ],
    },

    # ---------- Demo 4: 弹射起飞 + 搜索 + 返航(强校验:必有后续 skill) ----------
    "catapult_search_return": {
        "flow_id": "demo_catapult_search_return_001",
        "work_mode": 3,  # SEARCH_ONLY — 弹射后纯搜索,只验证强校验链路
        "skills": [
            {
                "skill_id": "takeoff_catapult_001",
                "skill_type": 100,
                "takeoff_subtype": 1,  # 弹射起飞
                "takeoff_altitude": 30.0,  # 相对 PX4 home,目标 alt = home_alt + 30
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                "skill_id": "search_001",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 6.0,
                "arrive_path": [
                    (50.0, 0.0, 30.0),
                ],
                "skill_area_path": [
                    (50.0, 50.0, 30.0),
                    (50.0, 150.0, 30.0),
                ],
            },
            {
                "skill_id": "return_001",
                "skill_type": 103,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 0.0, 30.0),
                ],
            },
        ],
    },

    # ---------- Demo 5: 地面起飞 + 搜索 (用于调试gimbal和目标定位) ----------
    "ground_search": {
        "flow_id": "demo_ground_search_001",
        "work_mode": 3,  # SEARCH_ONLY — 仅搜索
        "skills": [
            # 1) Takeoff (地面起飞, takeoff_subtype=0)
            {
                "skill_id": "takeoff_001",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,  # 相对 PX4 home,目标 alt = home_alt + 30
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],  # 起飞不需 arrive_path
                "skill_area_path": [],
            },
            # 2) Search(到达搜索区,识别目标)
            {
                "skill_id": "search_001",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 6.0,
                "arrive_path": [
                    (0.0, -30.0, 30.0),   # 原地起飞到 30m 后先飞往搜索线南端上方
                ],
                "skill_area_path": [
                    (-30.0, -30.0, 30.0),  # 搜索线南端 (目标在 y=10,这里在下边)
                    (-30.0, 10.0, 30.0),   # 正对目标 (y=10 与目标 x/y 一致) — 应该在这里看到目标
                    (-30.0, 30.0, 30.0),   # 越过目标继续搜索
                ],
            },
        ],
    },

    # ---------- Demo 6: 真实任务数据 — 起飞+集结+搜索+返航 ----------
    # 数据来自真实 GS 下发的 TaskFlow,航点为绝对 GPS 坐标
    # 参考点 (ref) = 起飞点 WP1: (36.0962811N, 114.3922342E, alt=0)
    # 已按 NED 偏移换算,publish_demo_flow 会自动覆盖 self.ref_lat/lon
    # 校验: 用上面 ref 反算回 GPS 应等于原始坐标 (允许 ~0.01m 误差)
    "b47c_normal": {
        "flow_id": "b47c_normal",
        "work_mode": 3,  # SEARCH_ONLY — 纯搜索,无打击
        "ref_lat": 36.0962811,
        "ref_lon": 114.39223419999998,
        "ref_alt": 0.0,
        "skills": [
            # 1) Takeoff — 原始 GPS: WP1=(36.0962811, 114.3922342, 30), WP2=(36.0965128, 114.3926923, 30)
            {
                "skill_id": "9bcd",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,
                "priority": 0,
                "cruise_speed": 0.0,
                "task_speed": 0.0,
                "arrive_path": [
                    (0.00, 0.00, 30.0),    # WP1 — 起飞点上空 (ref + 30m)
                    (25.71, 41.10, 30.0),  # WP2 — 起飞后过渡点 (约 48m 飞行)
                ],
                "skill_area_path": [],
                "params_json": '{"max_duration":900.0,"takeoff_height":30.0,"takeoff_time_interval":20.0}',
            },
            # 2) Gather — 原始 GPS: WP1=(36.0965128, 114.3926923, 30), WP2=(36.0967201, 114.3928568, 30)
            {
                "skill_id": "a0d6",
                "skill_type": 101,
                "takeoff_subtype": 0,
                "takeoff_altitude": 0.0,
                "priority": 0,
                "cruise_speed": 5.0,
                "task_speed": 5.0,
                "arrive_path": [
                    (25.71, 41.10, 30.0),  # WP1 — 接 takeoff WP2
                    (48.73, 55.86, 30.0),  # WP2 — 集结 HOVER 点
                ],
                "skill_area_path": [],
                "params_json": '{"gather_height":30.0,"gather_speed":5.0,"max_duration":900.0}',
            },
            # 3) Search — 原始 GPS: WP1=(36.0967201, 114.3928568, 30), WP2=(36.0964496, 114.3932106, 30)
            {
                "skill_id": "942b",
                "skill_type": 102,
                "takeoff_subtype": 0,
                "takeoff_altitude": 0.0,
                "priority": 0,
                "cruise_speed": 5.0,
                "task_speed": 5.0,
                "arrive_path": [
                    (48.73, 55.86, 30.0),  # WP1 — 接 gather WP2
                    (18.71, 87.61, 30.0),  # WP2 — 搜索终点
                ],
                "skill_area_path": [],
                "params_json": '{"max_duration":900.0,"safe_distance":10.0,"search_height":30.0,"search_speed":5.0}',
            },
            # 4) Return — 原始 GPS: WP1=(36.0964496, 114.3932106, 30), WP2=(36.0963810, 114.3925973, 30)
            {
                "skill_id": "bd3b",
                "skill_type": 103,
                "takeoff_subtype": 0,
                "takeoff_altitude": 0.0,
                "priority": 0,
                "cruise_speed": 5.0,
                "task_speed": 5.0,
                "arrive_path": [
                    (18.71, 87.61, 30.0),  # WP1 — 接 search WP2
                    (11.09, 32.58, 30.0),  # WP2 — 返航点(接近 home)
                ],
                "skill_area_path": [],
                "params_json": '{"max_duration":900.0,"return_height":30.0,"return_speed":5.0}',
            },
        ],
    },

    # ---------- Demo 7: 真实任务数据 — 纯起飞(单 skill)----------
    # 数据来自真实 GS 下发的 TaskFlow,航点为绝对 GPS 坐标
    # 参考点 (ref) = 起飞点 WP1: (36.0962778N, 114.3922288E, alt=0)
    "groupb1bc": {
        "flow_id": "groupb1bc",
        "work_mode": 3,  # SEARCH_ONLY — 占位,本 flow 只有 takeoff
        "ref_lat": 36.096277799999996,
        "ref_lon": 114.3922288,
        "ref_alt": 0.0,
        "skills": [
            # 1) Takeoff — 原始 GPS: WP1=(36.0962778, 114.3922288, 30), WP2=(36.0960795, 114.3922483, 30)
            {
                "skill_id": "b1bc",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,
                "priority": 0,
                "cruise_speed": 0.0,
                "task_speed": 0.0,
                "arrive_path": [
                    (0.00, 0.00, 30.0),     # WP1 — 起飞点上空 (ref + 30m)
                    (-22.01, 1.75, 30.0),   # WP2 — 起飞后过渡点 (约 22m 飞行)
                ],
                "skill_area_path": [],
                "params_json": '{"max_duration":900.0,"takeoff_height":30.0,"takeoff_time_interval":20.0}',
            },
        ],
    },

    # ============================================================================
    # 多机协同搜索航线(20-30 个搜索航点 + 起飞/集结/返航,路径有交集供避障触发)
    # 坐标系: 每机本地 NED(米),NED 原点 = 该机 PX4 home(GPS)。
    # 各机 home 在 SITL 启动时给不同 GPS,通常差 ~30m,搜索区(±50m)GPS 上有大量重叠。
    # ============================================================================

    # ---------- 多机 1: 东西向蛇形搜索 30 点 ----------
    # 6 条横线 y=[-50,-30,-10,10,30,50],每条 5 点 x=[-50,-25,0,25,50],相邻行反向
    "multi_uav0_search": {
        "flow_id": "multi_uav0_search_001",
        "work_mode": 3,  # SEARCH_ONLY
        "skills": [
            {
                "skill_id": "uav0_takeoff",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                "skill_id": "uav0_gather",
                "skill_type": 101,
                "priority": 100,
                "cruise_speed": 5.0,
                "task_speed": 4.0,
                "arrive_path": [],
                "skill_area_path": [(20.0, 0.0, 30.0),  # 集结到 home 东侧 20m
                ],
            },
            {
                "skill_id": "uav0_search",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 5.0,
                "arrive_path": [
                    (-50.0, -50.0, 30.0),  # 搜索起点:西南角
                ],
                "skill_area_path": [
                    # y=-50, 西→东
                    (-50.0, -50.0, 30.0), (-25.0, -50.0, 30.0), (0.0, -50.0, 30.0), (25.0, -50.0, 30.0), (50.0, -50.0, 30.0),
                    # y=-30, 东→西
                    (50.0, -30.0, 30.0), (25.0, -30.0, 30.0), (0.0, -30.0, 30.0), (-25.0, -30.0, 30.0), (-50.0, -30.0, 30.0),
                    # y=-10, 西→东
                    (-50.0, -10.0, 30.0), (-25.0, -10.0, 30.0), (0.0, -10.0, 30.0), (25.0, -10.0, 30.0), (50.0, -10.0, 30.0),
                    # y=10, 东→西
                    (50.0, 10.0, 30.0), (25.0, 10.0, 30.0), (0.0, 10.0, 30.0), (-25.0, 10.0, 30.0), (-50.0, 10.0, 30.0),
                    # y=30, 西→东
                    (-50.0, 30.0, 30.0), (-25.0, 30.0, 30.0), (0.0, 30.0, 30.0), (25.0, 30.0, 30.0), (50.0, 30.0, 30.0),
                    # y=50, 东→西
                    (50.0, 50.0, 30.0), (25.0, 50.0, 30.0), (0.0, 50.0, 30.0), (-25.0, 50.0, 30.0), (-50.0, 50.0, 30.0),
                ],
            },
            {
                "skill_id": "uav0_return",
                "skill_type": 103,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 0.0, 30.0),  # 返航到 home 上空
                ],
            },
        ],
    },

    # ---------- 多机 2: 南北向蛇形搜索 30 点(uav0 旋转 90°,交集更多)----------
    "multi_uav1_search": {
        "flow_id": "multi_uav1_search_001",
        "work_mode": 3,
        "skills": [
            {
                "skill_id": "uav1_takeoff",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                "skill_id": "uav1_gather",
                "skill_type": 101,
                "priority": 100,
                "cruise_speed": 5.0,
                "task_speed": 4.0,
                "arrive_path": [],
                "skill_area_path": [(-20.0, 0.0, 30.0),  # 集结到 home 西侧 20m(与 uav0 错开方向)
                ],
            },
            {
                "skill_id": "uav1_search",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 5.0,
                "arrive_path": [
                    (-50.0, -50.0, 30.0),  # 搜索起点:同 uav0 西南角
                ],
                "skill_area_path": [
                    # x=-50, 南→北
                    (-50.0, -50.0, 30.0), (-50.0, -25.0, 30.0), (-50.0, 0.0, 30.0), (-50.0, 25.0, 30.0), (-50.0, 50.0, 30.0),
                    # x=-30, 北→南
                    (-30.0, 50.0, 30.0), (-30.0, 25.0, 30.0), (-30.0, 0.0, 30.0), (-30.0, -25.0, 30.0), (-30.0, -50.0, 30.0),
                    # x=-10, 南→北
                    (-10.0, -50.0, 30.0), (-10.0, -25.0, 30.0), (-10.0, 0.0, 30.0), (-10.0, 25.0, 30.0), (-10.0, 50.0, 30.0),
                    # x=10, 北→南
                    (10.0, 50.0, 30.0), (10.0, 25.0, 30.0), (10.0, 0.0, 30.0), (10.0, -25.0, 30.0), (10.0, -50.0, 30.0),
                    # x=30, 南→北
                    (30.0, -50.0, 30.0), (30.0, -25.0, 30.0), (30.0, 0.0, 30.0), (30.0, 25.0, 30.0), (30.0, 50.0, 30.0),
                    # x=50, 北→南
                    (50.0, 50.0, 30.0), (50.0, 25.0, 30.0), (50.0, 0.0, 30.0), (50.0, -25.0, 30.0), (50.0, -50.0, 30.0),
                ],
            },
            {
                "skill_id": "uav1_return",
                "skill_type": 103,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 0.0, 30.0),
                ],
            },
        ],
    },

    # ---------- 多机 3: 螺旋向外搜索 30 点(半径 5→50) ----------
    # 5 圈 × 6 点 = 30 点;从 home 中心向外扩,与前两架搜索区中心重合
    "multi_uav2_search": {
        "flow_id": "multi_uav2_search_001",
        "work_mode": 3,
        "skills": [
            {
                "skill_id": "uav2_takeoff",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                "skill_id": "uav2_gather",
                "skill_type": 101,
                "priority": 100,
                "cruise_speed": 5.0,
                "task_speed": 4.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 20.0, 30.0),  # 集结到 home 北侧 20m
                ],
            },
            {
                "skill_id": "uav2_search",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 6.0,
                "task_speed": 4.0,
                "arrive_path": [
                    (10.0, 0.0, 30.0),  # 搜索起点:东向 10m
                ],
                "skill_area_path": [
                    # 圈1: r=10
                    (10.0, 0.0, 30.0), (5.0, 8.66, 30.0), (-5.0, 8.66, 30.0), (-10.0, 0.0, 30.0), (-5.0, -8.66, 30.0), (5.0, -8.66, 30.0),
                    # 圈2: r=20
                    (20.0, 0.0, 30.0), (10.0, 17.32, 30.0), (-10.0, 17.32, 30.0), (-20.0, 0.0, 30.0), (-10.0, -17.32, 30.0), (10.0, -17.32, 30.0),
                    # 圈3: r=30
                    (30.0, 0.0, 30.0), (15.0, 25.98, 30.0), (-15.0, 25.98, 30.0), (-30.0, 0.0, 30.0), (-15.0, -25.98, 30.0), (15.0, -25.98, 30.0),
                    # 圈4: r=40
                    (40.0, 0.0, 30.0), (20.0, 34.64, 30.0), (-20.0, 34.64, 30.0), (-40.0, 0.0, 30.0), (-20.0, -34.64, 30.0), (20.0, -34.64, 30.0),
                    # 圈5: r=50
                    (50.0, 0.0, 30.0), (25.0, 43.30, 30.0), (-25.0, 43.30, 30.0), (-50.0, 0.0, 30.0), (-25.0, -43.30, 30.0), (25.0, -43.30, 30.0),
                ],
            },
            {
                "skill_id": "uav2_return",
                "skill_type": 103,
                "priority": 100,
                "cruise_speed": 8.0,
                "task_speed": 8.0,
                "arrive_path": [],
                "skill_area_path": [
                    (0.0, 0.0, 30.0),
                ],
            },
        ],
    },

    # ============================================================================
    # 多机避障压测:三条轨迹都收敛到 **同一个 GPS 区域**(以 uav0_home 为共享原点),
    # 强制触发 proximity_threshold (15m)。
    #
    # 重要前提 — sitl_triple_run.sh 的 home 间距:
    #   uav0_home = (36.05801135, 114.5491114)
    #   uav1_home = (36.05813582, 114.54906363)   — uav0 北 13.82m,西 4.29m
    #   uav2_home = (36.05826582, 114.54908363)   — uav0 北 28.36m,西 2.49m
    #   共享 GPS 原点 = uav0_home
    #
    # 每机 NED 转换:本机 NED(x,y) 在 GPS 空间 = 本机_home + (x m 北, y m 东)
    # 要让 uav1/uav2 飞到 uav0_home,需要把它们各自的 NED 减去 home 偏移:
    #   uav1 飞到共享 (x, y) → uav1_NED = (x - 13.82, y + 4.29)
    #   uav2 飞到共享 (x, y) → uav2_NED = (x - 28.36, y + 2.49)
    # (第一版用各自 NED 偏移,导致 uav0/uav1 在 GPS 空间还差 14m,根本触发不到 threshold,已修正)
    #
    # 设计: 全部 8m × 8m 小方块,共享中心 = uav0_home,三机速度一致 → 同时刻在 12m 内
    # ============================================================================

    # ---------- UAV0 前向激光雷达楼体越障演示 ----------
    "lidar_building_uav0": {
        "flow_id": "lidar_building_uav0_001",
        "work_mode": 3,
        "skills": [
            {
                "skill_id": "lidar_demo_takeoff",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,
                "priority": 100,
                "cruise_speed": 5.0,
                "task_speed": 5.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                "skill_id": "lidar_demo_forward_flight",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 20.0,
                "task_speed": 20.0,
                "arrive_path": [],
                "skill_area_path": [(650.0, 0.0, 30.0)],
            },
        ],
    },

    # ---------- 避障压测 uav0:uav0_home 本机 NED 即共享坐标 ----------
    # 本机 NED(x,y) = 共享 (x, y) — uav0_home 在共享原点
    "avoid_test_uav0": {
        "flow_id": "avoid_test_uav0_001",
        "work_mode": 3,
        "skills": [
            {
                "skill_id": "uav0_takeoff",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,
                "priority": 100,
                "cruise_speed": 5.0,
                "task_speed": 5.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                # 集结点直接 = 共享原点 = uav0_home 上空 30m
                "skill_id": "uav0_search",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 5.0,
                "task_speed": 4.0,
                "arrive_path": [],
                "skill_area_path": [(0.0, 0.0, 30.0)],
            },
            # {
            #     # 顺时针方块 8m × 8m,中心 = uav0_home
            #     "skill_id": "uav0_search",
            #     "skill_type": 102,
            #     "priority": 100,
            #     "cruise_speed": 5.0,
            #     "task_speed": 4.0,
            #     "arrive_path": [],
            #     "skill_area_path": [
            #         (4.0, 0.0, 30.0),    # E
            #         (4.0, 4.0, 30.0),    # NE
            #         (0.0, 4.0, 30.0),    # N
            #         (-4.0, 4.0, 30.0),   # NW
            #         (-4.0, 0.0, 30.0),   # W
            #         (-4.0, -4.0, 30.0),  # SW
            #         (0.0, -4.0, 30.0),   # S
            #         (4.0, -4.0, 30.0),   # SE
            #         (4.0, 0.0, 30.0),    # 回 E
            #     ],
            # },
            # {
            #     "skill_id": "uav0_return",
            #     "skill_type": 103,
            #     "priority": 100,
            #     "cruise_speed": 5.0,
            #     "task_speed": 5.0,
            #     "arrive_path": [],
            #     "skill_area_path": [(0.0, 0.0, 30.0)],
            # },
        ],
    },

    # ---------- 避障压测 uav1:uav1_NED = 共享 (x - 13.82, y + 4.29) ----------
    "avoid_test_uav1": {
        "flow_id": "avoid_test_uav1_001",
        "work_mode": 3,
        "skills": [
            {
                "skill_id": "uav1_takeoff",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,
                "priority": 100,
                "cruise_speed": 5.0,
                "task_speed": 5.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                # 集结点:共享 (0, 0) → uav1_NED = (0 - 13.82, 0 + 4.29) = (-13.82, 4.29, 30)
                "skill_id": "uav1_searech",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 5.0,
                "task_speed": 4.0,
                "arrive_path": [],
                "skill_area_path": [(-13.82, 4.29, 30.0),
                                    (-13.82, 40.29, 30.0),
                                    (-20.82, 4.29, 30.0),],
            },
            # {
            #     # 逆时针方块 8m × 8m,中心 = uav0_home(共享原点)
            #     "skill_id": "uav1_search",
            #     "skill_type": 102,
            #     "priority": 100,
            #     "cruise_speed": 5.0,
            #     "task_speed": 4.0,
            #     "arrive_path": [],
            #     "skill_area_path": [
            #         (-9.82, 4.29, 30.0),    # 共享 (4, 0)   E
            #         (-9.82, 8.29, 30.0),    # 共享 (4, 4)   NE
            #         (-13.82, 8.29, 30.0),   # 共享 (0, 4)   N
            #         (-17.82, 8.29, 30.0),   # 共享 (-4, 4)  NW
            #         (-17.82, 4.29, 30.0),   # 共享 (-4, 0)  W
            #         (-17.82, 0.29, 30.0),   # 共享 (-4, -4) SW
            #         (-13.82, 0.29, 30.0),   # 共享 (0, -4)  S
            #         (-9.82, 0.29, 30.0),    # 共享 (4, -4)  SE
            #         (-9.82, 4.29, 30.0),    # 回 E
            #     ],
            # },
            # {
            #     "skill_id": "uav1_return",
            #     "skill_type": 103,
            #     "priority": 100,
            #     "cruise_speed": 5.0,
            #     "task_speed": 5.0,
            #     "arrive_path": [],
            #     "skill_area_path": [(-13.82, 4.29, 30.0)],
            # },
        ],
    },

    # ---------- 避障压测 uav2:uav2_NED = 共享 (x - 28.36, y + 2.49) ----------
    "avoid_test_uav2": {
        "flow_id": "avoid_test_uav2_001",
        "work_mode": 3,
        "skills": [
            {
                "skill_id": "uav2_takeoff",
                "skill_type": 106,
                "takeoff_subtype": 0,
                "takeoff_altitude": 30.0,
                "priority": 100,
                "cruise_speed": 5.0,
                "task_speed": 5.0,
                "arrive_path": [],
                "skill_area_path": [],
            },
            {
                # 集结点:共享 (0, 0) → uav2_NED = (0 - 28.36, 0 + 2.49) = (-28.36, 2.49, 30)
                "skill_id": "uav2_search",
                "skill_type": 102,
                "priority": 100,
                "cruise_speed": 5.0,
                "task_speed": 4.0,
                "arrive_path": [],
                "skill_area_path": [(-28.36, 2.49, 30.0),
                                    (-28.36, 20.49, 30.0),
                                    (-0.36, 2.49, 30.0)],
            },
            # {
            #     # 对角方块 8m × 8m,中心 = uav0_home,起点错开(从 SW 开始)
            #     "skill_id": "uav2_search",
            #     "skill_type": 102,
            #     "priority": 100,
            #     "cruise_speed": 5.0,
            #     "task_speed": 4.0,
            #     "arrive_path": [],
            #     "skill_area_path": [
            #         (-32.36, -1.51, 30.0),   # 共享 (-4, -4) SW  (起点)
            #         (-28.36, -1.51, 30.0),   # 共享 (0, -4)  S
            #         (-24.36, -1.51, 30.0),   # 共享 (4, -4)  SE
            #         (-24.36, 2.49, 30.0),    # 共享 (4, 0)   E
            #         (-24.36, 6.49, 30.0),    # 共享 (4, 4)   NE
            #         (-28.36, 6.49, 30.0),    # 共享 (0, 4)   N
            #         (-32.36, 6.49, 30.0),    # 共享 (-4, 4)  NW
            #         (-32.36, 2.49, 30.0),    # 共享 (-4, 0)  W
            #         (-32.36, -1.51, 30.0),   # 回 SW
            #     ],
            # },
            # {
            #     "skill_id": "uav2_return",
            #     "skill_type": 103,
            #     "priority": 100,
            #     "cruise_speed": 5.0,
            #     "task_speed": 5.0,
            #     "arrive_path": [],
            #     "skill_area_path": [(-28.36, 2.49, 30.0)],
            # },
        ],
    },
}


class GroundStationSimulator:
    def __init__(self, flow="", uav_ns="", device_id=0,
                 ref_lat=0.0, ref_lon=0.0, ref_alt=0.0,
                 gather_partners=""):
        self.flow = flow
        self.uav_ns = uav_ns
        self.device_id = device_id

        # === 期望集结伙伴 SN 列表(逗号分隔,如 "1-1,1-2,1-3")===
        # 仅对 skill_type=101 (Gather) 生效:会以 "devices_sn" 字段注入到 params_json,
        # mission_manager 端 mission_manager 解析后用作多机同步门控。
        # 单机/未设置 → 不注入,mission_manager 走 single_uav_arrived 老路径。
        self.gather_partners = [s.strip() for s in gather_partners.split(',') if s.strip()]
        if self.gather_partners:
            rospy.loginfo("[GS_Simulator] Gather partners: %s", self.gather_partners)

        # 参考 GPS 点(NED 局部坐标系原点):PX4 home 的延迟加载
        # 默认 0/0/0 表示"等 PX4 推 home 过来",见 _home_position_callback
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt = ref_alt

        # === 等待 PX4 home 真正到达(避免在 home 之前发布 TaskFlow 用错 ref 产生 ~km 级偏差)===
        # 只要带 uav_ns(SITL / 真实机场景),就强制等 PX4 home 到位再发布。
        # 注意:common.yaml 里有 ref_lat/lon 默认值(36.0588/114.557221),所以不能用
        # "ref_lat==0.0 and ref_lon==0.0" 来判断"launch 没显式传 ref",那样 common.yaml 默认值
        # 会让 wait_for_home_ = False → GS 用 common.yaml 的 fallback 发布 → waypoint 偏 730m。
        # 真正"launch 注入绝对 ref"的场景只有 b47c_normal / groupb1bc 这两条 flow,
        # 它们在 publish_demo_flow() 里会从 cfg["ref_lat"] 覆盖 self.ref_lat 再 rebuild,
        # 所以即便先等到了 PX4 home,后续也会被 cfg 内的绝对 ref 顶替,不受影响。
        self.wait_for_home_ = bool(uav_ns)
        self.is_home_received_ = False

        # === 唯一发布者:typed TaskFlow(打到 /<uav_ns>/mission/task_flow)
        tf_topic = "mission/task_flow"
        if uav_ns:
            tf_topic = "/{}/{}".format(uav_ns, tf_topic)
        self.task_flow_pub = rospy.Publisher(tf_topic, mus_msg.TaskFlow, queue_size=10, latch=True)
        rospy.loginfo("[GS_Simulator] TaskFlow topic: %s", tf_topic)

        # === typed WorkMode 发布器(告诉 mission_manager 任务模式,SEARCH_STRIKE/TRACK/ONLY)
        #   mission_manager 收不到 work_mode 就一直 IDLE,YOLO 命中时不会触发 strike/lock。
        wm_topic = "mission/work_mode"
        if uav_ns:
            wm_topic = "/{}/{}".format(uav_ns, wm_topic)
        self.work_mode_pub = rospy.Publisher(wm_topic, mus_msg.WorkMode, queue_size=10, latch=True)
        rospy.loginfo("[GS_Simulator] WorkMode topic: %s", wm_topic)

        # === 订阅 PX4 home 自动填充 ref_* — 替代 yaml 硬编码 ===
        if uav_ns:
            home_topic = "/{}/mavros/home_position/home".format(uav_ns)
            self.home_sub = rospy.Subscriber(
                home_topic, mavros_msgs.msg.HomePosition,
                self._home_position_callback, queue_size=1)
            rospy.loginfo("[GS_Simulator] Subscribed to PX4 home on %s", home_topic)

        rospy.loginfo("[GS_Simulator] Initialized flow=%s",
                      self.flow or "(none)")

    def _home_position_callback(self, msg):
        """PX4 home 一次性锁定 — 第一帧有效数据后写入 ref_lat/lon/alt"""
        lat = msg.geo.latitude
        lon = msg.geo.longitude
        alt = msg.geo.altitude
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return  # PX4 home 还没稳定(发 0/0)
        # 第一次有效帧写入;之后再收也不再覆盖
        self.ref_lat = lat
        self.ref_lon = lon
        self.ref_alt = alt
        self.is_home_received_ = True
        rospy.logwarn("[GS_Simulator] >>>> PX4 home loaded: lat=%.7f lon=%.7f alt=%.2f",
                        self.ref_lat, self.ref_lon, self.ref_alt)
        rospy.loginfo("[GS_Simulator] Ref GPS: lat=%.6f, lon=%.6f, alt=%.1f",
                      self.ref_lat, self.ref_lon, self.ref_alt)

    def ned_to_gps(self, north_m, east_m, up_m):
        """局部 NED 偏移 + 相对高度 → GPS (lat, lon, alt)"""
        d_lat = north_m / 111000.0
        d_lon = east_m / (111000.0 * cos(self.ref_lat * pi / 180.0))
        lat = self.ref_lat + d_lat
        lon = self.ref_lon + d_lon
        alt = up_m
        return lat, lon, alt

    def local_to_gps_waypoint(self, north_m, east_m, up_m):
        """把局部 NED + 高度航点转成 PoseStamped(GPS)"""
        lat, lon, alt = self.ned_to_gps(north_m, east_m, up_m)
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = lat
        pose.pose.position.y = lon
        pose.pose.position.z = alt
        pose.pose.orientation.w = 1.0
        return pose

    # ============== typed TaskFlow(唯一的 GS 输出通道)==============

    def build_task_flow(self, flow_name):
        """根据 flow_name 构造 multi_uav_strike/TaskFlow 消息"""
        if flow_name not in DEMO_FLOWS:
            rospy.logerr("[GS_Simulator] Unknown flow '%s', available: %s",
                         flow_name, list(DEMO_FLOWS.keys()))
            return None

        cfg = DEMO_FLOWS[flow_name]
        flow = mus_msg.TaskFlow()
        flow.flow_id = cfg["flow_id"]
        flow.issued_at_us = int(time.time() * 1e6)
        flow.device_id = self.device_id

        for skill_cfg in cfg["skills"]:
            s = mus_msg.Skill()
            s.skill_id = skill_cfg["skill_id"]
            s.skill_type = skill_cfg["skill_type"]
            s.takeoff_subtype = skill_cfg.get("takeoff_subtype", 0)
            s.takeoff_altitude = float(skill_cfg.get("takeoff_altitude", 0.0))
            s.priority = skill_cfg.get("priority", 100)
            s.cruise_speed = float(skill_cfg.get("cruise_speed", 8.0))
            s.task_speed = float(skill_cfg.get("task_speed", 8.0))
            s.formation = skill_cfg.get("formation", 0)

            # === 注入 devices_sn 到 gather skill 的 params_json ===
            #   mission_manager 端 parseDevicesSnFromParamsJson() 解出来用作多机同步名单
            #   单机/未设置时,跳过注入 → 老 single_uav_arrived 路径不受影响
            raw_params = skill_cfg.get("params_json", "")
            if self.gather_partners and s.skill_type == 101:
                try:
                    params = json.loads(raw_params) if raw_params else {}
                except ValueError:
                    rospy.logwarn("[GS_Simulator] Skill %s params_json not valid JSON, "
                                  "overwriting with devices_sn only: %s",
                                  s.skill_id, raw_params)
                    params = {}
                params["devices_sn"] = list(self.gather_partners)
                s.params_json = json.dumps(params)
                rospy.loginfo("[GS_Simulator] Injected devices_sn=%s into gather skill %s",
                              self.gather_partners, s.skill_id)
            else:
                s.params_json = raw_params

            # arrive_path
            for north_m, east_m, up_m in skill_cfg.get("arrive_path", []):
                pose = self.local_to_gps_waypoint(north_m, east_m, up_m)
                s.arrive_path.poses.append(pose)
            s.arrive_path.header.frame_id = "map"
            s.arrive_path.header.stamp = rospy.Time.now()

            # skill_area_path
            for north_m, east_m, up_m in skill_cfg.get("skill_area_path", []):
                pose = self.local_to_gps_waypoint(north_m, east_m, up_m)
                s.skill_area_path.poses.append(pose)
            s.skill_area_path.header.frame_id = "map"
            s.skill_area_path.header.stamp = rospy.Time.now()

            flow.skills.append(s)

        return flow

    def publish_demo_flow(self, flow_name):
        """发布选中的 demo flow"""
        flow = self.build_task_flow(flow_name)
        if flow is None:
            return

        cfg = DEMO_FLOWS[flow_name]
        work_mode = cfg.get("work_mode", 3)  # 默认 SEARCH_ONLY

        # === flow 内置 ref 覆盖 ===
        # 部分真实任务数据 (b47c_normal / groupb1bc) 的航点是绝对 GPS,
        # 参考点 = 该 flow takeoff 第一个航点,需要先覆盖 self.ref_* 再发布
        # 否则 NED→GPS 转换会基于 PX4 home,产生几百米的系统性偏差
        cfg_ref_lat = cfg.get("ref_lat")
        cfg_ref_lon = cfg.get("ref_lon")
        if cfg_ref_lat is not None and cfg_ref_lon is not None:
            self.ref_lat = cfg_ref_lat
            self.ref_lon = cfg_ref_lon
            if "ref_alt" in cfg:
                self.ref_alt = cfg["ref_alt"]
            # 重建 flow — 上面 build_task_flow 用的还是旧 ref_lat/lon
            flow = self.build_task_flow(flow_name)
            rospy.logwarn("[GS_Simulator] >>>> Flow ref override: lat=%.7f lon=%.7f alt=%.2f (rebuilt flow)",
                          self.ref_lat, self.ref_lon, self.ref_alt)

        rospy.logwarn("[GS_Simulator] >>>> Publishing demo flow '%s' (id=%s, %lu skills, work_mode=%u)",
                      flow_name, flow.flow_id, len(flow.skills), work_mode)
        for i, s in enumerate(flow.skills):
            rospy.logwarn("[GS_Simulator]   Skill[%d]: id=%s type=%u arrive=%lu skill=%lu task_speed=%.1f",
                          i, s.skill_id, s.skill_type,
                          len(s.arrive_path.poses), len(s.skill_area_path.poses),
                          s.task_speed)

        # === 先发 WorkMode,再发 TaskFlow ===
        # mission_manager 收到 WorkMode 时,typedWorkModeCallback 会立即把 current_work_mode_
        # 设成 SEARCH_STRIKE/SEARCH_TRACK/SEARCH_ONLY(对应 handleSearch* 分支)
        # 后续 YOLO 命中会按 work_mode 走对应路径:SEARCH_STRIKE 自动 triggerStrike()、
        # SEARCH_TRACK 等 attack_cmd、SEARCH_ONLY 只上报 DetectTarget。
        # WorkMode 必须在 TaskFlow 之前发,避免 mission_manager 在 first skill 是 takeoff 时
        # 误把 work_mode 缓存为 IDLE 后丢失(虽然当前 taskFlowCallback 不再覆盖 work_mode,
        # 但 typed WorkMode 先到位可让 post_takeoff_work_mode_ 兜底时不覆盖用户意图)。
        wm_msg = mus_msg.WorkMode()
        wm_msg.mode = work_mode
        self.work_mode_pub.publish(wm_msg)
        rospy.logwarn("[GS_Simulator] >>>> WorkMode=%u published.", work_mode)

        # 等 1s 让 subscriber 先建好连接
        rospy.sleep(1.0)
        self.task_flow_pub.publish(flow)
        rospy.logwarn("[GS_Simulator] >>>> Demo flow published.")

    def run(self):
        """运行:发了 flow 就退,空 flow 就 idle 等外部 GS"""
        if not self.flow:
            rospy.loginfo("[GS_Simulator] No flow param, idle and wait for external TaskFlow")
            rospy.spin()
            return

        # === 等待 PX4 home(launch 显式给了 ref_lat/lon 的 flow,例如 b47c/groupb1bc,跳过此步)===
        # 之前这里只 sleep(3.0),若 PX4 home 在 3s 内没到,会用 launch 默认 ref_lat/lon(0/0),
        # 后续 NED→GPS 转换基于经纬度 0,航点会被推到赤道附近,与 flight 端用的 PX4 home 错开几百~上千米。
        # 解决:用 rospy.wait_for_message 同步等一帧有效 home,最多 15s,避免启动卡死。
        if self.wait_for_home_:
            home_topic = "/{}/mavros/home_position/home".format(self.uav_ns)
            rospy.loginfo("[GS_Simulator] Waiting for PX4 home on %s (max 15s) before publishing...",
                          home_topic)
            try:
                home_msg = rospy.wait_for_message(home_topic, mavros_msgs.msg.HomePosition, timeout=15.0)
                if abs(home_msg.geo.latitude) > 1e-6 or abs(home_msg.geo.longitude) > 1e-6:
                    self.ref_lat = home_msg.geo.latitude
                    self.ref_lon = home_msg.geo.longitude
                    self.ref_alt = home_msg.geo.altitude
                    self.is_home_received_ = True
                    rospy.logwarn("[GS_Simulator] >>>> PX4 home pre-loaded: lat=%.7f lon=%.7f alt=%.2f",
                                  self.ref_lat, self.ref_lon, self.ref_alt)
                else:
                    rospy.logwarn("[GS_Simulator] PX4 home received but all zero, "
                                  "will proceed anyway (ref=%s)",
                                  (self.ref_lat, self.ref_lon, self.ref_alt))
            except rospy.ROSException as e:
                rospy.logwarn("[GS_Simulator] PX4 home wait timeout (%.1fs): %s. "
                              "Proceeding with ref=(%s, %s) — route may be offset.",
                              15.0, e, self.ref_lat, self.ref_lon)
        else:
            # 没有 uav_ns:GS 是哑节点,等外部 GS 推 TaskFlow,不需要 home
            rospy.loginfo("[GS_Simulator] No uav_ns (idle node), skipping PX4 home wait.")
            rospy.sleep(3.0)  # 保留原 sleep,让 subscriber 先到位

        self.publish_demo_flow(self.flow)
        rospy.loginfo("[GS_Simulator] Flow published, idle.")


def main():
    rospy.init_node('gs_simulator_node', anonymous=False)

    flow = rospy.get_param('~flow', '')             # demo flow 名称(空=等外部 GS)
    uav_ns = rospy.get_param('~uav_ns', '')         # UAV 命名空间(如 "uav0")
    device_id = rospy.get_param('~device_id', 0)    # 0=广播
    gather_partners = rospy.get_param('~gather_partners', '')  # 逗号分隔 SN,如 "1-1,1-2,1-3"

    ref_lat = rospy.get_param('~ref_lat', 0.0)
    ref_lon = rospy.get_param('~ref_lon', 0.0)
    ref_alt = rospy.get_param('~ref_alt', 0.0)

    rospy.loginfo("[GS_Simulator] Starting flow=%s",
                  flow or "(none)")
    rospy.loginfo("[GS_Simulator] Ref GPS: lat=%.6f, lon=%.6f, alt=%.1f",
                  ref_lat, ref_lon, ref_alt)

    simulator = GroundStationSimulator(
        flow=flow, uav_ns=uav_ns, device_id=device_id,
        ref_lat=ref_lat, ref_lon=ref_lon, ref_alt=ref_alt,
        gather_partners=gather_partners)
    simulator.run()


if __name__ == '__main__':
    main()
