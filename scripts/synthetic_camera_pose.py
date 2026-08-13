#!/usr/bin/env python3
"""synthetic_camera_pose.py — 替代 bag 里错的 camera/pose。

bag 的 153_manual_test_gimbal_0.bag 里 /uav0/camera/pose 全部 opt.z≈0
(光轴水平,不朝下),原因可能是飞行时云台没正确追踪地面目标。
target_estimator 反投影出射线与地面 plane 不相交 → proj_ok=false → cluster 都掉 (0,0,0)。

本节点订阅 uav_pose + bag 的 camera/pose(只取 stamp),
合成 "yaw=机头 yaw 或朝车, pitch=下视" 的 camera/pose 发布,
让 target_estimator 的反投影能正确落到地面。

camera/pose 的 orientation 字段含义与 gimbal_simulator::currentCameraWorldQuat() 一致:
  在 NWU 世界 + FLU 机体系下,云台 yaw 绕世界 Z,pitch 绕机体 Y(NWU y) 轴,
  R 的列 = [up_orth, right, opt]:
    opt = (cos(p)·cos(y), cos(p)·sin(y), -sin(p))   ← 光轴世界方向
  见 gimbal_simulator_node.cpp:691-721。

用法(launch 内):
  <node name="synthetic_camera_pose" pkg="multi_uav_strike"
        type="synthetic_camera_pose.py" output="screen">
    <param name="gimbal_pitch_deg" value="90.0"/>   <!-- 90 = 垂直下视, 45 = 45° 俯角 -->
    <param name="aim_at_ned_x" value="30.75"/>      <!-- 可选:瞄准地面 NED 点(覆盖 pitch_deg)-->
    <param name="aim_at_ned_y" value="1.0"/>
    <param name="aim_at_ned_z" value="-0.75"/>
    <param name="publish_hz" value="50.0"/>
  </node>

注意:本节点发布的 camera/pose 会与 bag 中同名 topic 冲突。
launch 必须从 rosbag play 的 topics 列表中移除 /uav0/camera/pose。
"""

import rospy
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped


def gimbal_camera_quat(yaw_nwu: float, pitch_nwu: float) -> np.ndarray:
    """与 gimbal_simulator::currentCameraWorldQuat() 完全一致的 R 构造 (xyzw quat)

    Args:
        yaw_nwu:   云台偏航 (NWU,rad), 从北 (x+) 起朝西 (y+) 为正
        pitch_nwu: 云台俯仰 (NWU,rad), 正值朝下 (光轴 z 向下)

    Returns:
        quaternion xyzw
    """
    cp = np.cos(pitch_nwu)
    sp = np.sin(pitch_nwu)
    cy = np.cos(yaw_nwu)
    sy = np.sin(yaw_nwu)

    opt = np.array([cp * cy, cp * sy, -sp])
    world_up = np.array([0.0, 0.0, 1.0])
    up_proj = world_up - opt * np.dot(world_up, opt)
    if np.linalg.norm(up_proj) < 1e-3:
        # 光轴几乎垂直 → 用 yaw 方向作 "上"
        up_proj = np.array([cy, sy, 0.0])
    up_proj = up_proj / np.linalg.norm(up_proj)

    right = np.cross(opt, up_proj)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([0.0, 1.0, 0.0])
    else:
        right = right / np.linalg.norm(right)

    up_orth = np.cross(right, opt)

    R = np.column_stack([up_orth, right, opt])
    # tf.transformations.quaternion_from_matrix expects 4x4 homogeneous
    R4 = np.eye(4)
    R4[:3, :3] = R
    return tft.quaternion_from_matrix(R4)  # xyzw


def uav_pose_to_world_state(x_enu: float, y_enu: float, z_enu: float,
                            q_enu_wxyz: np.ndarray):
    """mavros ENU (x,y,z) + 机体 FRD 姿态 → NWU (x,y,z) + FLU 机体系下的 (yaw_NWU)
    """
    # ENU → NWU
    x_nwu = y_enu
    y_nwu = -x_enu
    z_nwu = z_enu
    # yaw_NWU = yaw_ENU - π/2 (ENU 的 0 朝东, NWU 的 0 朝北)
    # 机体 FRD→FLU 不影响 yaw(只是机体系符号)
    # 标准 yaw from quaternion (mavros 给 FRD orientation in ENU world):
    #   yaw = atan2(2(wz+xy), 1-2(y²+z²))  (FRD body in ENU world)
    w, x, y, z = q_enu_wxyz
    yaw_enu = np.arctan2(2.0 * (w * z + x * y),
                         1.0 - 2.0 * (y * y + z * z))
    yaw_nwu = yaw_enu - np.pi / 2.0
    # wrap to [-π, π]
    while yaw_nwu > np.pi:
        yaw_nwu -= 2 * np.pi
    while yaw_nwu < -np.pi:
        yaw_nwu += 2 * np.pi
    return x_nwu, y_nwu, z_nwu, yaw_nwu


class SyntheticCameraPose:
    def __init__(self):
        rospy.init_node("synthetic_camera_pose")

        # ---- params ----
        self.gimbal_pitch_deg = float(rospy.get_param("~gimbal_pitch_deg", 90.0))
        self.min_pitch_deg = float(rospy.get_param("~min_pitch_deg", 30.0))  # 真实云台物理下限,防止 UAV 低于目标时抬头看天
        self.min_uav_alt_m = float(rospy.get_param("~min_uav_alt_m", 1.0))  # 起飞阶段(z<1m)不发布相机姿态,反投影与 z=0 平面无解
        self.aim_at_ned_x = rospy.get_param("~aim_at_ned_x", float("nan"))
        self.aim_at_ned_y = rospy.get_param("~aim_at_ned_y", float("nan"))
        self.aim_at_ned_z = rospy.get_param("~aim_at_ned_z", float("nan"))
        self.use_uav_yaw = bool(rospy.get_param("~use_uav_yaw", True))
        self.publish_hz = float(rospy.get_param("~publish_hz", 50.0))

        # ---- state ----
        self.last_uav_x_nwu = None
        self.last_uav_y_nwu = None
        self.last_uav_z_nwu = None
        self.last_uav_yaw_nwu = None
        self.last_bag_stamp = None  # last bag camera/pose stamp
        self.n_uav_pose_cb = 0

        # ---- I/O ----
        self.uav_pose_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped,
            self.uav_pose_cb, queue_size=10)
        self.bag_cam_sub = rospy.Subscriber(
            "camera/pose", PoseStamped,
            self.bag_cam_cb, queue_size=10)
        self.cam_pose_pub = rospy.Publisher(
            "camera/pose", PoseStamped, queue_size=10)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_hz), self.timer_cb)

        aim_str = (f"aim_at_NED=({self.aim_at_ned_x},{self.aim_at_ned_y},{self.aim_at_ned_z})"
                   if not (np.isnan(self.aim_at_ned_x) or np.isnan(self.aim_at_ned_y) or np.isnan(self.aim_at_ned_z))
                   else "no_aim")
        rospy.loginfo(
            "[synthetic_cam] pitch_deg=%.1f yaw_mode=%s %s hz=%.1f",
            self.gimbal_pitch_deg,
            "uav_heading" if self.use_uav_yaw else "fixed",
            aim_str, self.publish_hz)

    def uav_pose_cb(self, msg):
        """mavros PoseStamped (ENU+FRD) → 缓存 NWU 位置 + yaw_NWU"""
        p = msg.pose.position
        q = msg.pose.orientation
        x_n, y_n, z_n, yaw_n = uav_pose_to_world_state(
            p.x, p.y, p.z, np.array([q.w, q.x, q.y, q.z]))
        self.last_uav_x_nwu = x_n
        self.last_uav_y_nwu = y_n
        self.last_uav_z_nwu = z_n
        self.last_uav_yaw_nwu = yaw_n
        rospy.loginfo_throttle(2.0,
            "[synthetic_cam] uav_pose#%d  ENU=(%.2f,%.2f,%.2f)  NWU=(%.2f,%.2f,%.2f)  yaw_NWU=%.2fdeg",
            self.n_uav_pose_cb, p.x, p.y, p.z, x_n, y_n, z_n, np.degrees(yaw_n))
        self.n_uav_pose_cb += 1

    def bag_cam_cb(self, msg):
        """只记 stamp(我们的 publisher 复用 bag stamp 同步 yolo)"""
        self.last_bag_stamp = msg.header.stamp

    def timer_cb(self, event):
        if self.last_uav_x_nwu is None:
            return
        # 起飞阶段(z<min_uav_alt_m)不发布:相机光轴朝下,但 cam_pos.z()<0 时
        # ray 与 target_z_prior_=0 plane 反向相交(t<0),反投影退化。
        if self.last_uav_z_nwu < self.min_uav_alt_m:
            return

        x_n = self.last_uav_x_nwu
        y_n = self.last_uav_y_nwu
        z_n = self.last_uav_z_nwu

        # ---- 决定 yaw + pitch ----
        aim_x = self.aim_at_ned_x
        aim_y = self.aim_at_ned_y
        aim_z = self.aim_at_ned_z
        use_aim = (not np.isnan(aim_x) and not np.isnan(aim_y) and not np.isnan(aim_z))

        if use_aim:
            # car NED (a, b, c) → NWU (a, -b, -c)
            car_x_nwu = aim_x
            car_y_nwu = -aim_y
            car_z_nwu = -aim_z
            dx = car_x_nwu - x_n   # north
            dy = car_y_nwu - y_n   # west
            dz = car_z_nwu - z_n   # up
            horiz = np.hypot(dx, dy)
            yaw_nwu = np.arctan2(dy, dx)
            # pitch_NWU: 正值=朝下
            # car 在地面 (z<0), UAV 高空 z>0 → dz < 0
            # 我们要光轴朝 car → 光轴 dz_world = (car_z - uav_z)/|d|, < 0
            # opt.z = -sin(pitch) = dz_world < 0 → sin(pitch) > 0 → pitch > 0 ✓
            pitch_nwu = np.arctan2(-dz, horiz)
            # 真实云台物理下限:UAV 起飞瞬间(z≈0) car 在 z=+0.75 → dz>0 → 抬头看天,
            # 这种时刻反投影的射线与 z=0 plane 反向相交(target_z_prior_=0),完全没用。
            # 强制至少下视 min_pitch_deg°,与真云台一致。
            pitch_nwu = max(pitch_nwu, np.deg2rad(self.min_pitch_deg))
            rospy.loginfo_throttle(2.0,
                "[synthetic_cam] aim car from NWU(%.2f,%.2f,%.2f): "
                "d=(%.2f,%.2f,%.2f) yaw=%.2fdeg pitch=%.2fdeg",
                x_n, y_n, z_n, dx, dy, dz,
                np.degrees(yaw_nwu), np.degrees(pitch_nwu))
        else:
            pitch_nwu = np.deg2rad(self.gimbal_pitch_deg)
            yaw_nwu = (self.last_uav_yaw_nwu if self.use_uav_yaw else 0.0)

        # ---- 构造 quaternion (NWU + FLU 下的相机方向) ----
        q_xyzw = gimbal_camera_quat(yaw_nwu, pitch_nwu)

        # ---- publish ----
        out = PoseStamped()
        out.header.stamp = (self.last_bag_stamp
                            if self.last_bag_stamp is not None
                            else rospy.Time.now())
        out.header.frame_id = "world"
        out.pose.position.x = x_n
        out.pose.position.y = y_n
        out.pose.position.z = z_n
        out.pose.orientation.x = q_xyzw[0]
        out.pose.orientation.y = q_xyzw[1]
        out.pose.orientation.z = q_xyzw[2]
        out.pose.orientation.w = q_xyzw[3]
        self.cam_pose_pub.publish(out)


if __name__ == "__main__":
    try:
        SyntheticCameraPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass