#!/usr/bin/env python3
"""Reconstruct and diagnose an Attack/PN flight from one or more ROS1 bag files."""

import argparse
import csv
import math
import os
import statistics
import sys
from collections import defaultdict

try:
    import rosbag
except ImportError:
    print("error: Python cannot import rosbag; source the ROS and catkin workspaces first", file=sys.stderr)
    sys.exit(2)


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def norm3(vector):
    return math.sqrt(sum(component * component for component in vector))


def point_tuple(point):
    return (float(point.x), float(point.y), float(point.z))


def vector_tuple(vector):
    return (float(vector.x), float(vector.y), float(vector.z))


def enu_point_to_nwu(point):
    return (float(point.y), -float(point.x), float(point.z))


def enu_vector_to_ned(vector):
    return (float(vector.y), float(vector.x), -float(vector.z))


def twist_linear(message):
    twist = message.twist if hasattr(message, "twist") else message
    return twist.linear


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def finite(value):
    return value is not None and math.isfinite(value)


def format_number(value, digits=2):
    if not finite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def topic(namespace, suffix):
    return f"/{namespace}/{suffix.lstrip('/')}"


def add_event(events, stamp, kind, detail):
    events.append((stamp, kind, detail))


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Analyze PN geometry, frame consistency, descent feasibility, and strike-state "
            "transitions from ROS1 bags produced by single_uav_px4_tzs_ctl.launch."
        )
    )
    parser.add_argument("bags", nargs="+", help="bag file(s), in chronological order")
    parser.add_argument("--uav", default="uav0", help="UAV namespace (default: uav0)")
    parser.add_argument(
        "--max-vertical-speed",
        type=float,
        default=3.0,
        help="configured strike vertical limit in m/s (default: 3.0)",
    )
    parser.add_argument(
        "--csv",
        nargs="?",
        const="auto",
        help="write synchronized samples to CSV; optionally provide output path",
    )
    parser.add_argument(
        "--all-rosout",
        action="store_true",
        help="include matching INFO rosout records as well as WARN/ERROR records",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    namespace = args.uav.strip("/")
    if not namespace:
        print("error: --uav must not be empty", file=sys.stderr)
        return 2

    bag_paths = []
    for path in args.bags:
        if path.endswith(".active"):
            print(f"warning: skipping active bag {path}", file=sys.stderr)
            continue
        if not os.path.isfile(path):
            print(f"error: bag does not exist: {path}", file=sys.stderr)
            return 2
        bag_paths.append(path)
    if not bag_paths:
        print("error: no completed bag files supplied", file=sys.stderr)
        return 2

    t_pose = topic(namespace, "mavros/local_position/pose")
    t_velocity = topic(namespace, "mavros/local_position/velocity_local")
    t_setpoint = topic(namespace, "mavros/setpoint_velocity/cmd_vel_unstamped")
    t_target = topic(namespace, "target_estimated_pose")
    t_los = topic(namespace, "target_los_angle")
    t_pn = topic(namespace, "guidance/pn_debug")
    t_filter = topic(namespace, "guidance/target_filter_debug")
    t_enable = topic(namespace, "guidance/enable")
    t_mode = topic(namespace, "guidance/mode")
    t_speed = topic(namespace, "guidance/guidance_speed")
    t_mission = topic(namespace, "mission/mission_state")
    t_primary = topic(namespace, "mission/primary_target")
    t_cluster = topic(namespace, "cluster_states")
    t_waypoint_status = topic(namespace, "waypoint_executor/status")
    t_skill = topic(namespace, "waypoint_executor/skill")
    t_mavros_state = topic(namespace, "mavros/state")

    relevant_topics = {
        t_pose,
        t_velocity,
        t_setpoint,
        t_target,
        t_los,
        t_pn,
        t_filter,
        t_enable,
        t_mode,
        t_speed,
        t_mission,
        t_primary,
        t_cluster,
        t_waypoint_status,
        t_skill,
        t_mavros_state,
        "/rosout",
        "/rosout_agg",
    }

    available_topics = set()
    starts = []
    ends = []
    for path in bag_paths:
        try:
            with rosbag.Bag(path, "r") as bag:
                starts.append(bag.get_start_time())
                ends.append(bag.get_end_time())
                available_topics.update(bag.get_type_and_topic_info().topics.keys())
        except Exception as exc:
            print(f"error: cannot inspect {path}: {exc}", file=sys.stderr)
            return 2

    start_time = min(starts)
    end_time = max(ends)
    has_pn_debug = t_pn in available_topics

    cache = {
        "uav_nwu": None,
        "target_nwu": None,
        "actual_velocity_ned": None,
        "setpoint_ned": None,
        "filter_raw_ned": None,
        "filter_filtered_ned": None,
        "enabled": False,
        "mode": "unknown",
        "phase": None,
        "skill_type": None,
        "guidance_speed": None,
        "primary": None,
    }
    previous_event_values = {}
    primary_described = set()
    events = []
    rows = []
    last_geometry = None
    last_distance = None
    last_log_time = defaultdict(lambda: -1e9)

    def relative(stamp):
        return float(stamp) - start_time

    def append_sample(stamp, pn_message=None):
        nonlocal last_geometry, last_distance
        if has_pn_debug and pn_message is None:
            return

        if pn_message is not None:
            uav_nwu = point_tuple(pn_message.uav_position_nwu)
            target_nwu = point_tuple(pn_message.target_position_nwu)
            strategy_ned = vector_tuple(pn_message.strategy_velocity_ned)
            gated_ned = vector_tuple(pn_message.gated_velocity_ned)
            filtered_ned = vector_tuple(pn_message.filtered_velocity_ned)
            internal_yaw = float(pn_message.los_yaw)
            internal_pitch = float(pn_message.los_pitch)
            internal_yaw_rate = float(pn_message.los_yaw_rate)
            internal_pitch_rate = float(pn_message.los_pitch_rate)
            gate_weight = float(pn_message.pitch_gate_weight)
        else:
            uav_nwu = cache["uav_nwu"]
            target_nwu = cache["target_nwu"]
            strategy_ned = cache["filter_raw_ned"]
            gated_ned = cache["filter_raw_ned"]
            filtered_ned = cache["filter_filtered_ned"] or cache["setpoint_ned"]
            internal_yaw = None
            internal_pitch = None
            internal_yaw_rate = None
            internal_pitch_rate = None
            gate_weight = None

        if uav_nwu is None or target_nwu is None:
            return

        rel_nwu = tuple(target_nwu[index] - uav_nwu[index] for index in range(3))
        rel_ned = (rel_nwu[0], -rel_nwu[1], -rel_nwu[2])
        horizontal = math.hypot(rel_ned[0], rel_ned[1])
        distance = norm3(rel_ned)
        geom_yaw = math.atan2(rel_ned[1], rel_ned[0])
        geom_pitch = math.atan2(-rel_ned[2], horizontal)

        geom_yaw_rate = None
        geom_pitch_rate = None
        if last_geometry is not None:
            dt = stamp - last_geometry[0]
            if 1e-4 < dt <= 0.5:
                geom_yaw_rate = wrap_pi(geom_yaw - last_geometry[1]) / dt
                geom_pitch_rate = (geom_pitch - last_geometry[2]) / dt
        last_geometry = (stamp, geom_yaw, geom_pitch)

        closing_rate = None
        if last_distance is not None:
            dt = stamp - last_distance[0]
            if 1e-4 < dt <= 0.5:
                closing_rate = -(distance - last_distance[1]) / dt
        last_distance = (stamp, distance)

        command_ned = filtered_ned or cache["setpoint_ned"]
        alignment_deg = None
        command_horizontal = None
        command_down = None
        command_speed = None
        command_descent_deg = None
        lead_deg = None
        if command_ned is not None:
            command_speed = norm3(command_ned)
            command_horizontal = math.hypot(command_ned[0], command_ned[1])
            command_down = command_ned[2]
            command_descent_deg = math.degrees(math.atan2(command_down, command_horizontal))
            if command_speed > 1e-6 and distance > 1e-6:
                cosine = sum(command_ned[i] * rel_ned[i] for i in range(3)) / (
                    command_speed * distance
                )
                alignment_deg = math.degrees(math.acos(clamp(cosine, -1.0, 1.0)))
            if command_horizontal > 1e-6:
                lead_deg = math.degrees(wrap_pi(math.atan2(command_ned[1], command_ned[0]) - geom_yaw))

        actual_ned = cache["actual_velocity_ned"]
        actual_speed = norm3(actual_ned) if actual_ned is not None else None
        actual_horizontal = (
            math.hypot(actual_ned[0], actual_ned[1]) if actual_ned is not None else None
        )
        actual_down = actual_ned[2] if actual_ned is not None else None
        actual_descent_deg = (
            math.degrees(math.atan2(actual_down, actual_horizontal))
            if actual_ned is not None
            else None
        )

        setpoint_frame_error = None
        if cache["setpoint_ned"] is not None and filtered_ned is not None:
            setpoint_frame_error = norm3(
                tuple(cache["setpoint_ned"][i] - filtered_ned[i] for i in range(3))
            )

        rows.append(
            {
                "time": relative(stamp),
                "phase": cache["phase"],
                "skill_type": cache["skill_type"],
                "enabled": cache["enabled"],
                "mode": cache["mode"],
                "guidance_speed": cache["guidance_speed"],
                "uav_n": uav_nwu[0],
                "uav_w": uav_nwu[1],
                "uav_u": uav_nwu[2],
                "target_n": target_nwu[0],
                "target_w": target_nwu[1],
                "target_u": target_nwu[2],
                "distance": distance,
                "horizontal_distance": horizontal,
                "geom_los_yaw_deg": math.degrees(geom_yaw),
                "geom_los_pitch_deg": math.degrees(geom_pitch),
                "geom_los_yaw_rate_deg_s": (
                    math.degrees(geom_yaw_rate) if geom_yaw_rate is not None else None
                ),
                "geom_los_pitch_rate_deg_s": (
                    math.degrees(geom_pitch_rate) if geom_pitch_rate is not None else None
                ),
                "pn_los_yaw_deg": math.degrees(internal_yaw) if internal_yaw is not None else None,
                "pn_los_pitch_deg": (
                    math.degrees(internal_pitch) if internal_pitch is not None else None
                ),
                "pn_los_yaw_rate_deg_s": (
                    math.degrees(internal_yaw_rate) if internal_yaw_rate is not None else None
                ),
                "pn_los_pitch_rate_deg_s": (
                    math.degrees(internal_pitch_rate) if internal_pitch_rate is not None else None
                ),
                "strategy_n": strategy_ned[0] if strategy_ned is not None else None,
                "strategy_e": strategy_ned[1] if strategy_ned is not None else None,
                "strategy_d": strategy_ned[2] if strategy_ned is not None else None,
                "gated_n": gated_ned[0] if gated_ned is not None else None,
                "gated_e": gated_ned[1] if gated_ned is not None else None,
                "gated_d": gated_ned[2] if gated_ned is not None else None,
                "filtered_n": filtered_ned[0] if filtered_ned is not None else None,
                "filtered_e": filtered_ned[1] if filtered_ned is not None else None,
                "filtered_d": filtered_ned[2] if filtered_ned is not None else None,
                "setpoint_n": cache["setpoint_ned"][0] if cache["setpoint_ned"] else None,
                "setpoint_e": cache["setpoint_ned"][1] if cache["setpoint_ned"] else None,
                "setpoint_d": cache["setpoint_ned"][2] if cache["setpoint_ned"] else None,
                "actual_n": actual_ned[0] if actual_ned is not None else None,
                "actual_e": actual_ned[1] if actual_ned is not None else None,
                "actual_d": actual_ned[2] if actual_ned is not None else None,
                "command_speed": command_speed,
                "command_horizontal_speed": command_horizontal,
                "command_down_speed": command_down,
                "command_descent_angle_deg": command_descent_deg,
                "actual_speed": actual_speed,
                "actual_horizontal_speed": actual_horizontal,
                "actual_down_speed": actual_down,
                "actual_descent_angle_deg": actual_descent_deg,
                "command_to_los_angle_deg": alignment_deg,
                "horizontal_lead_angle_deg": lead_deg,
                "closing_rate": closing_rate,
                "pitch_gate_weight": gate_weight,
                "setpoint_frame_error": setpoint_frame_error,
            }
        )

    ordered_topics = sorted(relevant_topics & available_topics)
    for path in bag_paths:
        try:
            with rosbag.Bag(path, "r") as bag:
                for current_topic, message, bag_stamp in bag.read_messages(topics=ordered_topics):
                    stamp = bag_stamp.to_sec()

                    if current_topic == t_pose:
                        cache["uav_nwu"] = enu_point_to_nwu(message.pose.position)
                        append_sample(stamp)
                    elif current_topic == t_velocity:
                        cache["actual_velocity_ned"] = enu_vector_to_ned(twist_linear(message))
                    elif current_topic == t_setpoint:
                        cache["setpoint_ned"] = enu_vector_to_ned(twist_linear(message))
                    elif current_topic == t_target:
                        cache["target_nwu"] = point_tuple(message.pose.position)
                    elif current_topic == t_filter and getattr(message, "source", "") == "velocity":
                        cache["filter_raw_ned"] = vector_tuple(message.vel_raw)
                        cache["filter_filtered_ned"] = vector_tuple(message.vel_filtered)
                    elif current_topic == t_pn:
                        append_sample(stamp, message)
                    elif current_topic == t_enable:
                        value = bool(message.data)
                        if previous_event_values.get("enable") != value:
                            add_event(events, stamp, "GUIDANCE", f"enable={str(value).lower()}")
                            previous_event_values["enable"] = value
                        cache["enabled"] = value
                    elif current_topic == t_mode:
                        value = str(message.data)
                        if previous_event_values.get("mode") != value:
                            add_event(events, stamp, "GUIDANCE", f"mode={value}")
                            previous_event_values["mode"] = value
                        cache["mode"] = value
                    elif current_topic == t_speed:
                        value = float(message.data)
                        if previous_event_values.get("speed") != value:
                            add_event(events, stamp, "GUIDANCE", f"speed={value:.2f} m/s")
                            previous_event_values["speed"] = value
                        cache["guidance_speed"] = value
                    elif current_topic == t_mission:
                        value = (int(message.skill_type), int(message.state), int(message.phase))
                        if previous_event_values.get("mission") != value:
                            add_event(
                                events,
                                stamp,
                                "MISSION",
                                f"skill_type={value[0]} state={value[1]} phase={value[2]}",
                            )
                            previous_event_values["mission"] = value
                        cache["skill_type"] = value[0]
                        cache["phase"] = value[2]
                    elif current_topic == t_primary:
                        value = int(message.data)
                        if previous_event_values.get("primary") != value:
                            add_event(events, stamp, "TARGET", f"primary_cluster={value}")
                            previous_event_values["primary"] = value
                        cache["primary"] = value
                    elif current_topic == t_cluster:
                        primary = cache["primary"]
                        if primary and primary not in primary_described:
                            for cluster in message.targets:
                                if int(cluster.cluster_id) == primary:
                                    add_event(
                                        events,
                                        stamp,
                                        "TARGET",
                                        "cluster={} label={} nwu=({:.1f},{:.1f},{:.1f}) conf={:.2f}".format(
                                            primary,
                                            cluster.label,
                                            cluster.ned_x,
                                            cluster.ned_y,
                                            cluster.ned_alt,
                                            cluster.confidence,
                                        ),
                                    )
                                    primary_described.add(primary)
                                    break
                    elif current_topic == t_waypoint_status:
                        value = (
                            int(message.phase),
                            int(message.arrive_idx),
                            int(message.arrive_total),
                            int(message.skill_idx),
                            int(message.skill_total),
                        )
                        if previous_event_values.get("waypoint") != value:
                            add_event(
                                events,
                                stamp,
                                "WAYPOINT",
                                "phase={} arrive={}/{} skill={}/{}".format(*value),
                            )
                            previous_event_values["waypoint"] = value
                    elif current_topic == t_skill:
                        if int(message.skill_type) == 105:
                            waypoints = [
                                "({:.7f},{:.7f},{:.1f})".format(
                                    pose.pose.position.x,
                                    pose.pose.position.y,
                                    pose.pose.position.z,
                                )
                                for pose in message.skill_area_path.poses
                            ]
                            add_event(
                                events,
                                stamp,
                                "ATTACK_SKILL",
                                "id={} cruise={:.1f} task={:.1f} WP={}".format(
                                    message.skill_id,
                                    message.cruise_speed,
                                    message.task_speed,
                                    ", ".join(waypoints) if waypoints else "[]",
                                ),
                            )
                    elif current_topic == t_mavros_state:
                        value = (bool(message.connected), bool(message.armed), str(message.mode))
                        if previous_event_values.get("mavros") != value:
                            add_event(
                                events,
                                stamp,
                                "PX4",
                                f"connected={value[0]} armed={value[1]} mode={value[2]}",
                            )
                            previous_event_values["mavros"] = value
                    elif current_topic in ("/rosout", "/rosout_agg"):
                        level = int(getattr(message, "level", 0))
                        text = str(getattr(message, "msg", ""))
                        keywords = (
                            "STRIKE",
                            "Strike",
                            "strike",
                            "Intercept PN",
                            "TARGET_LOST",
                            "Target switched",
                            "WP1",
                            "WP2",
                            "Guidance ENABLED",
                            "Guidance DISABLED",
                        )
                        if (level >= 4 or args.all_rosout) and any(key in text for key in keywords):
                            key = (getattr(message, "name", ""), text)
                            if stamp - last_log_time[key] >= 0.5:
                                add_event(events, stamp, "ROSOUT", text.replace("\n", " | "))
                                last_log_time[key] = stamp
        except Exception as exc:
            print(f"error: cannot read {path}: {exc}", file=sys.stderr)
            return 2

    print("=== Strike bag analysis ===")
    print("bags:")
    for path in bag_paths:
        print(f"  - {path}")
    print(f"namespace: /{namespace}")
    print(f"bag span: {end_time - start_time:.2f} s")
    print(f"PN debug topic: {'present' if has_pn_debug else 'missing (using derived geometry)'}")

    required = [t_pose, t_target, t_setpoint, t_mission, t_enable, t_mode]
    missing = [name for name in required if name not in available_topics]
    if missing:
        print("missing core topics:")
        for name in missing:
            print(f"  - {name}")

    print("\n=== Event timeline ===")
    if events:
        for stamp, kind, detail in sorted(events, key=lambda item: item[0]):
            print(f"{relative(stamp):9.3f}s  {kind:12s}  {detail}")
    else:
        print("no state-transition events found")

    strike_rows = [
        row
        for row in rows
        if (row["enabled"] and row["mode"] == "strike") or row["phase"] == 5
    ]
    print("\n=== PN/strike metrics ===")
    if not strike_rows:
        print("no strike-guidance samples found (need guidance enable+mode or MissionState phase=5)")
    else:
        distances = [row["distance"] for row in strike_rows]
        pitches = [row["geom_los_pitch_deg"] for row in strike_rows]
        alignments = [
            row["command_to_los_angle_deg"]
            for row in strike_rows
            if finite(row["command_to_los_angle_deg"])
        ]
        closing_rates = [
            row["closing_rate"] for row in strike_rows if finite(row["closing_rate"])
        ]
        down_commands = [
            row["command_down_speed"]
            for row in strike_rows
            if finite(row["command_down_speed"])
        ]
        frame_errors = [
            row["setpoint_frame_error"]
            for row in strike_rows
            if finite(row["setpoint_frame_error"])
        ]
        duration = strike_rows[-1]["time"] - strike_rows[0]["time"]
        backwards = sum(1 for value in alignments if value > 90.0)
        target_below_no_descent = sum(
            1
            for row in strike_rows
            if row["geom_los_pitch_deg"] < -5.0
            and finite(row["command_down_speed"])
            and row["command_down_speed"] <= 0.05
        )
        retreat = [value for value in closing_rates if value < -0.2]
        vertical_limited = sum(
            1
            for value in down_commands
            if abs(value) >= 0.95 * args.max_vertical_speed
        )

        print(f"window: {strike_rows[0]['time']:.3f}s .. {strike_rows[-1]['time']:.3f}s ({duration:.2f}s)")
        print(
            "distance: {} -> {} m, min={} m".format(
                format_number(distances[0]),
                format_number(distances[-1]),
                format_number(min(distances)),
            )
        )
        print(
            "LOS pitch: start={} deg, median={} deg, max_abs={} deg".format(
                format_number(pitches[0], 1),
                format_number(statistics.median(pitches), 1),
                format_number(max(abs(value) for value in pitches), 1),
            )
        )
        if alignments:
            print(
                "command/LOS alignment: median={} deg, p95={} deg, backwards={}/{}".format(
                    format_number(statistics.median(alignments), 1),
                    format_number(percentile(alignments, 0.95), 1),
                    backwards,
                    len(alignments),
                )
            )
        if closing_rates:
            print(
                "closing rate: median={} m/s, p05={} m/s, retreat_samples={}/{}".format(
                    format_number(statistics.median(closing_rates)),
                    format_number(percentile(closing_rates, 0.05)),
                    len(retreat),
                    len(closing_rates),
                )
            )
        if down_commands:
            print(
                "vertical command: min={} max={} m/s (NED down+), at_limit={}/{}".format(
                    format_number(min(down_commands)),
                    format_number(max(down_commands)),
                    vertical_limited,
                    len(down_commands),
                )
            )
        print(
            f"target below but descent suppressed: {target_below_no_descent}/{len(strike_rows)} samples"
        )
        if frame_errors:
            print(
                "NED-filter -> ENU-setpoint round-trip error: median={} m/s, max={} m/s".format(
                    format_number(statistics.median(frame_errors), 4),
                    format_number(max(frame_errors), 4),
                )
            )

        representative_pitch = abs(statistics.median(pitches))
        if representative_pitch >= 0.1:
            tangent = math.tan(math.radians(min(representative_pitch, 89.9)))
            max_horizontal = args.max_vertical_speed / tangent if tangent > 1e-6 else float("inf")
            max_path_speed = math.hypot(max_horizontal, args.max_vertical_speed)
            print(
                "vertical-limit geometry at |pitch|={}: max horizontal={} m/s, path speed={} m/s "
                "when preserving LOS direction".format(
                    format_number(representative_pitch, 1),
                    format_number(max_horizontal),
                    format_number(max_path_speed),
                )
            )

        warnings = []
        if backwards:
            warnings.append("some commands point more than 90 deg away from the target; inspect frame conversion")
        if target_below_no_descent > 0.05 * len(strike_rows):
            warnings.append("descent is frequently suppressed while the target is below")
        if retreat and len(retreat) > 0.05 * max(1, len(closing_rates)):
            warnings.append("distance increases in a material fraction of strike samples")
        if frame_errors and max(frame_errors) > 0.05:
            warnings.append("published MAVROS setpoint does not match the recorded filtered NED command")
        if vertical_limited > 0.5 * max(1, len(down_commands)):
            warnings.append("vertical limit dominates the strike; horizontal attack speed must be reduced")
        if warnings:
            print("diagnostic warnings:")
            for warning in warnings:
                print(f"  - {warning}")
        else:
            print("diagnostic warnings: none triggered by numeric thresholds")

    if args.csv:
        csv_path = args.csv
        if csv_path == "auto":
            base = os.path.splitext(os.path.basename(bag_paths[0]))[0]
            csv_path = os.path.join(os.path.dirname(os.path.abspath(bag_paths[0])), base + ".strike.csv")
        if rows:
            with open(csv_path, "w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)
            print(f"\nCSV: {csv_path} ({len(rows)} samples)")
        else:
            print("\nCSV not written: no synchronized samples", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
