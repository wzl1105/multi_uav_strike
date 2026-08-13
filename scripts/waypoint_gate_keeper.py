#!/usr/bin/env python3
"""waypoint_gate_keeper.py

强制让 target_estimator_node 的 isFirstWaypointGateOpen() 返回 true,绕过
对真实 mission_manager / waypoint_executor 节点的依赖。

target_estimator_node.cpp 中两道闸:
  1. missionStateCallback (line 751-804):
     cluster_skill_active_ = (state==1 AND skill_type==102) OR
                              (search_area_first_waypoint_reached_ AND
                               state==1 AND skill_type==105)
  2. updateFirstWaypointLatch (line 610-627):
     search_area_first_waypoint_reached_ ←  only when
       waypoint_status_received_ AND
       latest_waypoint_status_.skill_id == current_skill_id_ AND
       latest_waypoint_status_.skill_total > 0 AND
       latest_waypoint_status_.skill_idx >= 1

最终 isFirstWaypointGateOpen (line 629-641) 才放行 YOLO。

bag 153_manual_test_gimbal_0.bag 的 mission_state 全空(skill_type=-1,state=0),
waypoint_executor/status 根本没录 → 没有这条节点,target_estimator 静默丢弃所有 YOLO。
"""

import rospy
from multi_uav_strike.msg import MissionState, WaypointStatus

SKILL_ID = "iter_test"  # 与 waypoint_status.skill_id 对齐


def build_mission_state():
    msg = MissionState()
    msg.skill_flow_id = SKILL_ID
    msg.skill_id = SKILL_ID
    msg.skill_type = 102       # SEARCH_AREA,触发 cluster_skill_active_
    msg.state = 1               # EXECUTING
    msg.phase = 3              # WAYPOINT_FOLLOW (与 bag 当时实际 phase 对齐)
    msg.attack_arrival_time_us = 0
    return msg


def build_waypoint_status():
    msg = WaypointStatus()
    msg.phase = 2              # PHASE_SKILL_AREA
    msg.flow_id = SKILL_ID
    msg.skill_id = SKILL_ID    # 必须与 MissionState.skill_id 一致
    msg.arrive_idx = 0
    msg.arrive_total = 0
    msg.skill_idx = 1          # >= 1 触发 latch
    msg.skill_total = 2        # > 0 触发 latch
    return msg


def main():
    rospy.init_node("waypoint_gate_keeper", anonymous=False, log_level=rospy.INFO)
    pub_ms = rospy.Publisher("/uav0/mission/mission_state", MissionState,
                             queue_size=1, latch=True)
    pub_ws = rospy.Publisher("/uav0/waypoint_executor/status", WaypointStatus,
                             queue_size=1, latch=True)

    # 先 latch 一条,确保 target_estimator 起来就能收到
    pub_ms.publish(build_mission_state())
    pub_ws.publish(build_waypoint_status())
    rospy.loginfo("[gate_keeper] latched skill_id=%s state=1 type=102 "
                  "skill_idx=1/2", SKILL_ID)

    rate = rospy.Rate(1.0)  # 1Hz,与原 mission_state 发布频率一致
    while not rospy.is_shutdown():
        pub_ms.publish(build_mission_state())
        pub_ws.publish(build_waypoint_status())
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass