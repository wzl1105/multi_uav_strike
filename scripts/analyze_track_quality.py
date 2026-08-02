#!/usr/bin/env python3
"""Analyze waypoint line tracking, yaw stability, and motion smoothness from ROS1 bags."""

import argparse
import csv
import math
import os
import re
import statistics
import sys
from bisect import bisect_right
from dataclasses import dataclass

try:
    import rosbag
except ImportError:
    print(
        "error: Python cannot import rosbag; source the ROS and catkin workspaces first",
        file=sys.stderr,
    )
    sys.exit(2)


NAN = float("nan")
PHASE_ARRIVE = 1
PHASE_SKILL_AREA = 2
PHASE_NAMES = {
    PHASE_ARRIVE: "ARRIVE",
    PHASE_SKILL_AREA: "SKILL_AREA",
}


@dataclass
class BagInfo:
    path: str
    start: float
    end: float
    topics: set


@dataclass
class Segment:
    sequence: int
    skill_id: str
    phase: int
    index: int
    total: int
    t_start: float
    t_end: float
    first_observed_active: bool
    p0: object = None
    p1: object = None
    p0_source: str = "unknown"
    p1_spread: float = NAN
    skill_type: object = None
    speed_setpoint: float = NAN

    @property
    def label(self):
        return "{}[{}]".format(PHASE_NAMES.get(self.phase, str(self.phase)), self.index)


class StepCursor:
    def __init__(self, stream):
        self.stream = stream
        self.index = -1

    def value_at(self, stamp, max_age=None):
        while self.index + 1 < len(self.stream) and self.stream[self.index + 1][0] <= stamp:
            self.index += 1
        if self.index < 0:
            return None
        sample_stamp, value = self.stream[self.index]
        if max_age is not None and stamp - sample_stamp > max_age:
            return None
        return value


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def finite(value):
    return value is not None and math.isfinite(value)


def percentile(values, fraction):
    values = sorted(value for value in values if finite(value))
    if not values:
        return NAN
    index = fraction * (len(values) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return values[lower]
    weight = index - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def value_stats(values):
    values = [value for value in values if finite(value)]
    if not values:
        return {"median": NAN, "p95": NAN, "max": NAN, "rms": NAN}
    return {
        "median": statistics.median(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
        "rms": math.sqrt(sum(value * value for value in values) / len(values)),
    }


def vector_norm(vector):
    if vector is None or not all(finite(component) for component in vector):
        return NAN
    return math.sqrt(sum(component * component for component in vector))


def horizontal_norm(vector):
    if vector is None or not finite(vector[0]) or not finite(vector[1]):
        return NAN
    return math.hypot(vector[0], vector[1])


def degrees(value):
    return math.degrees(value) if finite(value) else NAN


def format_number(value, digits=2):
    return "n/a" if not finite(value) else "{:.{}f}".format(value, digits)


def topic(namespace, suffix):
    return "/{}/{}".format(namespace.strip("/"), suffix.lstrip("/"))


def enu_vector_to_ned(vector):
    return (float(vector.y), float(vector.x), -float(vector.z))


def enu_point_to_ned(point):
    return (float(point.y), float(point.x), -float(point.z))


def yaw_ned_from_enu_quaternion(quaternion):
    norm_sq = (
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm_sq < 0.5:
        return NAN
    sqrt_half = 0.7071067811865475
    qw = sqrt_half * (quaternion.w + quaternion.z)
    qx = sqrt_half * (quaternion.x + quaternion.y)
    qy = sqrt_half * (quaternion.x - quaternion.y)
    qz = sqrt_half * (quaternion.w - quaternion.z)
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def twist_of(message):
    return message.twist if hasattr(message, "twist") else message


def median_filter_scalars(values, radius=2):
    filtered = []
    for index in range(len(values)):
        window = [
            values[neighbor]
            for neighbor in range(max(0, index - radius), min(len(values), index + radius + 1))
            if finite(values[neighbor])
        ]
        filtered.append(statistics.median(window) if window else NAN)
    return filtered


def median_filter_vectors(vectors, radius=2):
    components = list(zip(*vectors)) if vectors else [(), (), ()]
    filtered_components = [median_filter_scalars(list(values), radius) for values in components]
    return list(zip(*filtered_components)) if vectors else []


def central_difference_vectors(times, vectors, max_span=0.6):
    result = [(NAN, NAN, NAN) for _ in vectors]
    for index in range(1, len(vectors) - 1):
        dt = times[index + 1] - times[index - 1]
        if dt <= 1e-4 or dt > max_span:
            continue
        before = vectors[index - 1]
        after = vectors[index + 1]
        if not all(finite(value) for value in before + after):
            continue
        result[index] = tuple((after[axis] - before[axis]) / dt for axis in range(3))
    return result


def robust_kinematics(stream):
    if not stream:
        return []
    times = [stamp for stamp, _ in stream]
    vectors = [(item["n"], item["e"], item["d"]) for _, item in stream]
    filtered_velocity = median_filter_vectors(vectors, radius=2)
    acceleration = central_difference_vectors(times, filtered_velocity)
    filtered_acceleration = median_filter_vectors(acceleration, radius=2)
    jerk = central_difference_vectors(times, filtered_acceleration)
    return [
        {
            "t": times[index],
            "v": filtered_velocity[index],
            "a": acceleration[index],
            "j": jerk[index],
        }
        for index in range(len(times))
    ]


def unwrap_angles(angles):
    if not angles:
        return []
    result = [angles[0]]
    for angle in angles[1:]:
        if not finite(angle) or not finite(result[-1]):
            result.append(angle)
        else:
            result.append(result[-1] + wrap_pi(angle - result[-1]))
    return result


def robust_angle_rates(times, angles):
    if len(times) != len(angles):
        raise ValueError("times and angles must have equal length")
    unwrapped = unwrap_angles(angles)
    filtered = median_filter_scalars(unwrapped, radius=2)
    result = [NAN for _ in times]
    for index in range(1, len(times) - 1):
        dt = times[index + 1] - times[index - 1]
        if dt <= 1e-4 or dt > 0.6:
            continue
        if finite(filtered[index - 1]) and finite(filtered[index + 1]):
            result[index] = (filtered[index + 1] - filtered[index - 1]) / dt
    return result


def reversal_metrics(samples, threshold, min_interval=0.25):
    state = 0
    reversal_times = []
    last_reversal = -1e9
    valid_times = []
    for stamp, value in samples:
        if not finite(value):
            continue
        valid_times.append(stamp)
        new_state = 1 if value > threshold else (-1 if value < -threshold else 0)
        if new_state == 0:
            continue
        if state != 0 and new_state != state and stamp - last_reversal >= min_interval:
            reversal_times.append(stamp)
            last_reversal = stamp
        state = new_state
    if len(valid_times) < 2:
        return 0, NAN, NAN
    duration = valid_times[-1] - valid_times[0]
    per_10s = 10.0 * len(reversal_times) / duration if duration > 1e-3 else NAN
    half_periods = [
        reversal_times[index] - reversal_times[index - 1]
        for index in range(1, len(reversal_times))
    ]
    period = 2.0 * statistics.median(half_periods) if half_periods else NAN
    return len(reversal_times), per_10s, period


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Analyze waypoint cross-track error, yaw oscillation, and acceleration/jerk "
            "from one or more ROS1 bag files."
        )
    )
    parser.add_argument("bags", nargs="+", help="completed bag file(s); split bags may be unordered")
    parser.add_argument("--uav", help="UAV namespace; auto-detected when exactly one is present")
    parser.add_argument("--output-dir", help="output directory (default: first bag directory)")
    parser.add_argument("--prefix", help="output filename prefix (default: first bag basename)")
    parser.add_argument("--plot", action="store_true", help="write a PNG overview (requires matplotlib)")

    parser.add_argument("--l1-min", type=float, default=3.0, help="controller L1_min in metres")
    parser.add_argument("--l1-period", type=float, default=0.6, help="controller L1_period in seconds")
    parser.add_argument(
        "--final-approach-threshold",
        type=float,
        default=2.5,
        help="distance where final P-control starts (metres)",
    )
    parser.add_argument(
        "--xte-damping-gain",
        type=float,
        default=0.0,
        help=(
            "XTE bearing damping gain; correction = atan2(-XTE, L1) * gain (radians). "
            "0 disables; >0 adds cross-track damping that scales with speed via L1."
        ),
    )
    parser.add_argument(
        "--l1-min-period",
        type=float,
        default=0.0,
        help=(
            "speed-aware L1_min replacement; L1_min_eff = max(L1_min, speed*l1_min_period). "
            "0 keeps fixed L1_min as the floor; >0 lets L1 grow with speed at low speed too."
        ),
    )
    parser.add_argument(
        "--replay-gain",
        nargs="*",
        default=[],
        metavar="GAIN:PERIOD",
        help=(
            "one or more 'GAIN:PERIOD' pairs to replay against the same bag, "
            "e.g. '1.0:0.8 1.5:1.0'. Each pair overrides --xte-damping-gain and "
            "--l1-min-period; emits a <prefix>.track_replay.csv comparing all variants."
        ),
    )
    parser.add_argument(
        "--settling-time",
        type=float,
        default=2.0,
        help="time excluded after each waypoint transition (seconds)",
    )
    parser.add_argument("--yaw-kp", type=float, default=3.0, help="waypoint yaw P gain")
    parser.add_argument("--yaw-deadband", type=float, default=0.05, help="yaw deadband in radians")
    parser.add_argument("--yaw-max-rate", type=float, default=1.0, help="yaw-rate limit in rad/s")
    parser.add_argument("--yaw-speed-gate", type=float, default=0.5, help="yaw speed gate in m/s")

    parser.add_argument("--xte-p95-limit", type=float, default=0.5, help="XTE P95 warning limit (m)")
    parser.add_argument(
        "--yaw-error-p95-limit-deg",
        type=float,
        default=8.0,
        help="desired-yaw error P95 warning limit (deg)",
    )
    parser.add_argument(
        "--yaw-reversal-limit",
        type=float,
        default=2.0,
        help="actual yaw-rate reversals per 10 s warning limit",
    )
    parser.add_argument(
        "--accel-p95-limit",
        type=float,
        default=3.0,
        help="actual horizontal acceleration P95 warning limit (m/s^2)",
    )
    parser.add_argument(
        "--jerk-p95-limit",
        type=float,
        default=15.0,
        help="actual horizontal jerk P95 warning limit (m/s^3)",
    )
    parser.add_argument(
        "--xte-hysteresis",
        type=float,
        default=0.05,
        help="XTE sign-change hysteresis (m)",
    )
    parser.add_argument(
        "--yaw-rate-reversal-threshold-deg-s",
        type=float,
        default=5.0,
        help="minimum yaw-rate amplitude counted as a reversal (deg/s)",
    )
    return parser.parse_args()


def inspect_bags(paths):
    infos = []
    for path in paths:
        if path.endswith(".active"):
            print("warning: skipping active bag {}".format(path), file=sys.stderr)
            continue
        if not os.path.isfile(path):
            raise ValueError("bag does not exist: {}".format(path))
        try:
            with rosbag.Bag(path, "r") as bag:
                infos.append(
                    BagInfo(
                        path=os.path.abspath(path),
                        start=float(bag.get_start_time()),
                        end=float(bag.get_end_time()),
                        topics=set(bag.get_type_and_topic_info().topics.keys()),
                    )
                )
        except Exception as exc:
            raise ValueError("cannot inspect {}: {}".format(path, exc))
    if not infos:
        raise ValueError("no completed bag files supplied")
    infos.sort(key=lambda info: (info.start, info.path))
    return infos


def discover_namespace(infos, requested):
    available = set().union(*(info.topics for info in infos))
    if requested:
        namespace = requested.strip("/")
        if not namespace:
            raise ValueError("--uav must not be empty")
        required = topic(namespace, "waypoint_executor/status")
        if required not in available:
            raise ValueError("{} is missing; available UAV namespaces: {}".format(required, ", ".join(sorted(discover_namespaces(available))) or "none"))
        return namespace
    candidates = discover_namespaces(available)
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise ValueError("cannot discover a UAV namespace from waypoint_executor/status topics")
    raise ValueError("multiple UAV namespaces found ({}); specify --uav".format(", ".join(candidates)))


def discover_namespaces(available_topics):
    pattern = re.compile(r"^/([^/]+)/waypoint_executor/status$")
    return sorted(match.group(1) for name in available_topics for match in [pattern.match(name)] if match)


def topic_map(namespace):
    return {
        "pose": topic(namespace, "mavros/local_position/pose"),
        "velocity": topic(namespace, "mavros/local_position/velocity_local"),
        "command": topic(namespace, "mavros/setpoint_velocity/cmd_vel_unstamped"),
        "imu": topic(namespace, "mavros/imu/data"),
        "status": topic(namespace, "waypoint_executor/status"),
        "target": topic(namespace, "waypoint_executor/current_target"),
        "skill": topic(namespace, "waypoint_executor/skill"),
        "control": topic(namespace, "waypoint_executor/control"),
        "guidance_enable": topic(namespace, "guidance/enable"),
        "avoidance": topic(namespace, "avoidance/cmd"),
    }


def deduplicate_stream(stream):
    stream.sort(key=lambda sample: sample[0])
    result = []
    for sample in stream:
        if result and abs(sample[0] - result[-1][0]) < 1e-9:
            result[-1] = sample
        else:
            result.append(sample)
    return result


def load_streams(infos, topics):
    reverse_topics = {name: key for key, name in topics.items()}
    streams = {key: [] for key in topics}
    selected = sorted(set(topics.values()) & set().union(*(info.topics for info in infos)))

    for info in infos:
        try:
            with rosbag.Bag(info.path, "r") as bag:
                for current_topic, message, bag_stamp in bag.read_messages(topics=selected):
                    stamp = float(bag_stamp.to_sec())
                    key = reverse_topics[current_topic]
                    if key == "pose":
                        n, e, d = enu_point_to_ned(message.pose.position)
                        streams[key].append(
                            (
                                stamp,
                                {
                                    "n": n,
                                    "e": e,
                                    "d": d,
                                    "yaw": yaw_ned_from_enu_quaternion(message.pose.orientation),
                                },
                            )
                        )
                    elif key in ("velocity", "command"):
                        twist = twist_of(message)
                        n, e, d = enu_vector_to_ned(twist.linear)
                        value = {"n": n, "e": e, "d": d}
                        if key == "command":
                            value["yaw_rate"] = -float(twist.angular.z)
                        streams[key].append((stamp, value))
                    elif key == "imu":
                        streams[key].append((stamp, {"yaw_rate": -float(message.angular_velocity.z)}))
                    elif key == "status":
                        streams[key].append(
                            (
                                stamp,
                                {
                                    "phase": int(message.phase),
                                    "flow_id": str(message.flow_id),
                                    "skill_id": str(message.skill_id),
                                    "arrive_idx": int(message.arrive_idx),
                                    "arrive_total": int(message.arrive_total),
                                    "skill_idx": int(message.skill_idx),
                                    "skill_total": int(message.skill_total),
                                },
                            )
                        )
                    elif key == "target":
                        streams[key].append(
                            (
                                stamp,
                                {
                                    "n": float(message.pose.position.x),
                                    "e": float(message.pose.position.y),
                                    "d": float(message.pose.position.z),
                                },
                            )
                        )
                    elif key == "skill":
                        streams[key].append(
                            (
                                stamp,
                                {
                                    "skill_id": str(message.skill_id),
                                    "skill_type": int(message.skill_type),
                                    "cruise_speed": float(message.cruise_speed),
                                    "task_speed": float(message.task_speed),
                                },
                            )
                        )
                    elif key == "control":
                        streams[key].append((stamp, str(message.data).strip().lower()))
                    elif key == "guidance_enable":
                        streams[key].append((stamp, bool(message.data)))
                    elif key == "avoidance":
                        speed_scale = float(getattr(message, "speed_scale", 1.0))
                        vertical_bias = float(getattr(message, "vertical_bias", 0.0))
                        reverse = bool(getattr(message, "reverse_allowed", False))
                        streams[key].append(
                            (
                                stamp,
                                {
                                    "active": speed_scale < 0.999 or abs(vertical_bias) > 1e-3 or reverse,
                                    "speed_scale": speed_scale,
                                    "vertical_bias": vertical_bias,
                                    "reverse": reverse,
                                },
                            )
                        )
        except Exception as exc:
            raise ValueError("cannot read {}: {}".format(info.path, exc))

    return {key: deduplicate_stream(stream) for key, stream in streams.items()}


def active_status_key(status):
    phase = status["phase"]
    if phase == PHASE_ARRIVE:
        index = status["arrive_idx"]
        total = status["arrive_total"]
    elif phase == PHASE_SKILL_AREA:
        index = status["skill_idx"]
        total = status["skill_total"]
    else:
        return None
    if total <= 0 or index >= total:
        return None
    return (status["skill_id"], phase, index, total)


def build_status_intervals(status_stream, bag_end):
    intervals = []
    current = None
    current_start = None
    current_payload = None
    current_first = False
    seen_status = False

    for stamp, status in status_stream:
        key = active_status_key(status)
        if key == current:
            seen_status = True
            continue
        if current is not None:
            intervals.append((current_start, stamp, current_payload, current_first))
        current = key
        current_start = stamp if key is not None else None
        current_payload = status if key is not None else None
        current_first = key is not None and not seen_status
        seen_status = True

    if current is not None:
        intervals.append((current_start, bag_end, current_payload, current_first))
    return intervals


def nearest_at_or_before(stream, stamp, max_age=None):
    if not stream:
        return None
    times = [sample[0] for sample in stream]
    index = bisect_right(times, stamp) - 1
    if index < 0:
        return None
    if max_age is not None and stamp - times[index] > max_age:
        return None
    return stream[index]


def matching_skill(skill_stream, skill_id, stamp):
    candidates = [
        sample
        for sample in skill_stream
        if sample[1]["skill_id"] == skill_id and sample[0] <= stamp + 0.5
    ]
    if not candidates:
        candidates = [sample for sample in skill_stream if sample[1]["skill_id"] == skill_id]
    return max(candidates, key=lambda sample: sample[0]) if candidates else None


def median_target(target_stream, start, end):
    values = [value for stamp, value in target_stream if start <= stamp < end]
    if not values:
        return None, NAN
    p1 = tuple(statistics.median(item[axis] for item in values) for axis in ("n", "e", "d"))
    spread = max(math.hypot(value["n"] - p1[0], value["e"] - p1[1]) for value in values)
    return p1, spread


def reconstruct_segments(streams, bag_start, bag_end):
    intervals = build_status_intervals(streams["status"], bag_end)
    segments = []

    for sequence, (start, end, status, first_observed) in enumerate(intervals):
        phase = status["phase"]
        index = status["arrive_idx"] if phase == PHASE_ARRIVE else status["skill_idx"]
        total = status["arrive_total"] if phase == PHASE_ARRIVE else status["skill_total"]
        segment = Segment(
            sequence=sequence,
            skill_id=status["skill_id"],
            phase=phase,
            index=index,
            total=total,
            t_start=start,
            t_end=end,
            first_observed_active=first_observed,
        )

        segment.p1, segment.p1_spread = median_target(streams["target"], start, end)
        skill_sample = matching_skill(streams["skill"], segment.skill_id, start)
        if skill_sample:
            _, skill = skill_sample
            segment.skill_type = skill["skill_type"]
            segment.speed_setpoint = (
                skill["cruise_speed"] if phase == PHASE_ARRIVE else skill["task_speed"]
            )

        if index > 0:
            previous = next(
                (
                    candidate
                    for candidate in reversed(segments)
                    if candidate.skill_id == segment.skill_id
                    and candidate.phase == phase
                    and candidate.index == index - 1
                    and candidate.p1 is not None
                ),
                None,
            )
            if previous:
                segment.p0 = previous.p1
                segment.p0_source = "previous_target"
        else:
            latch_stamp = start
            if first_observed:
                if skill_sample and bag_start - 1e-6 <= skill_sample[0] <= start + 0.5:
                    latch_stamp = skill_sample[0]
                else:
                    latch_stamp = None
            if latch_stamp is not None:
                pose_sample = nearest_at_or_before(streams["pose"], latch_stamp, max_age=0.5)
                if pose_sample:
                    pose = pose_sample[1]
                    segment.p0 = (pose["n"], pose["e"], pose["d"])
                    segment.p0_source = "phase_entry_pose"

        segments.append(segment)
    return segments


def build_execution_events(streams):
    events = []
    for stamp, _ in streams["skill"]:
        events.append((stamp, True, "skill"))
    for stamp, command in streams["control"]:
        if command in ("stop", "pause"):
            events.append((stamp, False, command))
        elif command == "resume":
            events.append((stamp, True, command))
    for stamp, enabled in streams["guidance_enable"]:
        if enabled:
            events.append((stamp, False, "guidance"))
    events.sort(key=lambda event: (event[0], 0 if event[1] else 1))
    return [(stamp, {"active": active, "source": source}) for stamp, active, source in events]


def segment_geometry(segment, pose, command, args):
    empty = {
        "segment_length": NAN,
        "segment_bearing": NAN,
        "xte": NAN,
        "along": NAN,
        "distance_to_target": NAN,
        "l1": NAN,
        "desired_yaw": NAN,
        "terminal": False,
        "steady": False,
    }
    if segment.p0 is None or segment.p1 is None:
        return empty

    p0n, p0e, _ = segment.p0
    p1n, p1e, _ = segment.p1
    seg_dn = p1n - p0n
    seg_de = p1e - p0e
    segment_length = math.hypot(seg_dn, seg_de)
    dn_target = p1n - pose["n"]
    de_target = p1e - pose["e"]
    distance_to_target = math.hypot(dn_target, de_target)
    if segment_length < 0.1:
        desired_yaw = math.atan2(de_target, dn_target)
        result = dict(empty)
        result.update(
            {
                "segment_length": segment_length,
                "segment_bearing": desired_yaw,
                "distance_to_target": distance_to_target,
                "desired_yaw": desired_yaw,
                "terminal": distance_to_target < args.final_approach_threshold,
            }
        )
        return result

    unit_n = seg_dn / segment_length
    unit_e = seg_de / segment_length
    rel_n = pose["n"] - p0n
    rel_e = pose["e"] - p0e
    along = rel_n * unit_n + rel_e * unit_e
    xte = unit_n * rel_e - unit_e * rel_n
    segment_bearing = math.atan2(unit_e, unit_n)

    control_speed = segment.speed_setpoint
    if not finite(control_speed) or control_speed <= 0.0:
        control_speed = horizontal_norm(command) if command else NAN
    if finite(control_speed):
        l1_min_eff = (
            max(args.l1_min, control_speed * args.l1_min_period)
            if args.l1_min_period > 0.0
            else args.l1_min
        )
        l1 = max(l1_min_eff, control_speed * args.l1_period)
    else:
        l1 = args.l1_min
    terminal = distance_to_target < args.final_approach_threshold

    if terminal:
        desired_yaw = math.atan2(de_target, dn_target)
    else:
        target_along = clamp(along + l1, 0.0, segment_length)
        lookahead_n = p0n + unit_n * target_along
        lookahead_e = p0e + unit_e * target_along
        desired_yaw = math.atan2(lookahead_e - pose["e"], lookahead_n - pose["n"])
        if (
            segment.skill_type == 105
            and segment.phase == PHASE_SKILL_AREA
            and segment.index >= 1
        ):
            desired_yaw = math.atan2(de_target, dn_target)
        # XTE damping: shift desired bearing by atan2(-XTE, L1) * gain.
        # At XTE=0 the correction is 0; as XTE grows the correction approaches
        # ±90° * gain, but the atan2 form naturally limits the rate of change
        # so the controller stays stable. The L1 divisor makes the same gain
        # work at both low speed (L1≈3 m → strong correction) and high speed
        # (L1≈9 m → gentle correction).
        if args.xte_damping_gain > 0.0 and finite(xte):
            xte_correction = math.atan2(-xte, max(l1, 1.0)) * args.xte_damping_gain
            desired_yaw = wrap_pi(desired_yaw + xte_correction)

    entry_margin = min(l1, 0.25 * segment_length)
    steady = (
        along >= entry_margin
        and along <= segment_length - args.final_approach_threshold
        and not terminal
    )
    return {
        "segment_length": segment_length,
        "segment_bearing": segment_bearing,
        "xte": xte,
        "along": along,
        "distance_to_target": distance_to_target,
        "l1": l1,
        "desired_yaw": desired_yaw,
        "terminal": terminal,
        "steady": steady,
    }


def build_sample_rows(streams, segments, bag_start, args):
    if not streams["pose"]:
        return []

    velocity_cursor = StepCursor(streams["velocity"])
    command_cursor = StepCursor(streams["command"])
    imu_cursor = StepCursor(streams["imu"])
    avoidance_cursor = StepCursor(streams["avoidance"])
    execution_events = build_execution_events(streams)
    execution_cursor = StepCursor(execution_events)

    pose_times = [stamp for stamp, _ in streams["pose"]]
    pose_yaws = [pose["yaw"] for _, pose in streams["pose"]]
    pose_yaw_rates = robust_angle_rates(pose_times, pose_yaws)

    rows = []
    segment_index = 0
    for pose_index, (stamp, pose) in enumerate(streams["pose"]):
        while segment_index < len(segments) and stamp >= segments[segment_index].t_end:
            segment_index += 1
        if segment_index >= len(segments):
            break
        segment = segments[segment_index]
        if stamp < segment.t_start:
            continue

        execution = execution_cursor.value_at(stamp)
        executor_active = execution["active"] if execution is not None else True
        if not executor_active:
            continue

        velocity = velocity_cursor.value_at(stamp, max_age=0.35)
        command = command_cursor.value_at(stamp, max_age=0.2)
        imu = imu_cursor.value_at(stamp, max_age=0.2)
        avoidance = avoidance_cursor.value_at(stamp, max_age=1.0)
        avoidance_active = bool(avoidance and avoidance["active"])
        geometry = segment_geometry(segment, pose, command, args)

        desired_yaw = geometry["desired_yaw"]
        yaw_error_desired = (
            wrap_pi(desired_yaw - pose["yaw"])
            if finite(desired_yaw) and finite(pose["yaw"])
            else NAN
        )
        yaw_error_segment = (
            wrap_pi(geometry["segment_bearing"] - pose["yaw"])
            if finite(geometry["segment_bearing"]) and finite(pose["yaw"])
            else NAN
        )
        command_speed = horizontal_norm(
            (command["n"], command["e"], command["d"]) if command else None
        )
        expected_yaw_rate = 0.0
        if (
            finite(yaw_error_desired)
            and finite(command_speed)
            and command_speed > args.yaw_speed_gate
            and abs(yaw_error_desired) > args.yaw_deadband
        ):
            expected_yaw_rate = clamp(
                args.yaw_kp * yaw_error_desired,
                -args.yaw_max_rate,
                args.yaw_max_rate,
            )

        actual_vector = (
            (velocity["n"], velocity["e"], velocity["d"])
            if velocity
            else (NAN, NAN, NAN)
        )
        command_vector = (
            (command["n"], command["e"], command["d"])
            if command
            else (NAN, NAN, NAN)
        )
        actual_speed = horizontal_norm(actual_vector)
        course = (
            math.atan2(actual_vector[1], actual_vector[0])
            if finite(actual_speed) and actual_speed > 0.2
            else NAN
        )
        steady = (
            geometry["steady"]
            and stamp >= segment.t_start + args.settling_time
            and not avoidance_active
        )

        rows.append(
            {
                "t_abs": stamp,
                "time_s": stamp - bag_start,
                "segment_seq": segment.sequence,
                "skill_id": segment.skill_id,
                "skill_type": segment.skill_type,
                "phase": segment.phase,
                "phase_name": PHASE_NAMES.get(segment.phase, str(segment.phase)),
                "index": segment.index,
                "label": segment.label,
                "steady": steady,
                "terminal": geometry["terminal"],
                "avoidance_active": avoidance_active,
                "pose_n": pose["n"],
                "pose_e": pose["e"],
                "pose_d": pose["d"],
                "xte_m": geometry["xte"],
                "along_m": geometry["along"],
                "distance_to_target_m": geometry["distance_to_target"],
                "segment_length_m": geometry["segment_length"],
                "l1_m": geometry["l1"],
                "segment_bearing": geometry["segment_bearing"],
                "desired_yaw": desired_yaw,
                "actual_yaw": pose["yaw"],
                "yaw_error_desired": yaw_error_desired,
                "yaw_error_segment": yaw_error_segment,
                "desired_yaw_rate": NAN,
                "cmd_yaw_rate": command["yaw_rate"] if command else NAN,
                "expected_yaw_rate": expected_yaw_rate,
                "actual_yaw_rate": imu["yaw_rate"] if imu else pose_yaw_rates[pose_index],
                "pose_yaw_rate": pose_yaw_rates[pose_index],
                "actual_vn": actual_vector[0],
                "actual_ve": actual_vector[1],
                "actual_vd": actual_vector[2],
                "actual_speed_h": actual_speed,
                "cmd_vn": command_vector[0],
                "cmd_ve": command_vector[1],
                "cmd_vd": command_vector[2],
                "cmd_speed_h": command_speed,
                "course": course,
            }
        )

    rows_by_segment = {}
    for row in rows:
        rows_by_segment.setdefault(row["segment_seq"], []).append(row)
    for segment_rows in rows_by_segment.values():
        times = [row["t_abs"] for row in segment_rows]
        desired = [row["desired_yaw"] for row in segment_rows]
        rates = robust_angle_rates(times, desired)
        for row, rate in zip(segment_rows, rates):
            row["desired_yaw_rate"] = rate
    return rows


def flag_at_time(segment_rows, stamp):
    if not segment_rows:
        return False
    times = [row["t_abs"] for row in segment_rows]
    index = bisect_right(times, stamp) - 1
    if index < 0:
        index = 0
    if abs(times[index] - stamp) > 0.2:
        return False
    return bool(segment_rows[index]["steady"])


def select_kinematics(kinematics, segment, segment_rows):
    selected = []
    for sample in kinematics:
        if segment.t_start <= sample["t"] < segment.t_end and flag_at_time(segment_rows, sample["t"]):
            selected.append(sample)
    return selected


def command_notch_at(command_stream, boundary):
    if len(command_stream) < 3:
        return False, NAN, NAN
    times = [sample[0] for sample in command_stream]
    first = max(1, bisect_right(times, boundary - 0.15) - 1)
    last = min(len(command_stream) - 1, bisect_right(times, boundary + 0.15))
    for index in range(first, last):
        current = command_stream[index][1]
        current_vector = (current["n"], current["e"], current["d"])
        if horizontal_norm(current_vector) >= 0.1:
            continue
        before_index = index - 1
        after_index = index + 1
        before = command_stream[before_index]
        after = command_stream[after_index]
        if boundary - before[0] > 0.2 or after[0] - boundary > 0.2:
            continue
        before_vector = (before[1]["n"], before[1]["e"], before[1]["d"])
        after_vector = (after[1]["n"], after[1]["e"], after[1]["d"])
        if horizontal_norm(before_vector) > 0.5 and horizontal_norm(after_vector) > 0.5:
            delta_before = vector_norm(tuple(current_vector[axis] - before_vector[axis] for axis in range(3)))
            delta_after = vector_norm(tuple(after_vector[axis] - current_vector[axis] for axis in range(3)))
            return True, times[index], max(delta_before, delta_after)
    return False, NAN, NAN


def decompose_kinematics(samples, segment):
    if segment.p0 is None or segment.p1 is None:
        return {}
    dn = segment.p1[0] - segment.p0[0]
    de = segment.p1[1] - segment.p0[1]
    length = math.hypot(dn, de)
    if length < 0.1:
        return {}
    unit_n = dn / length
    unit_e = de / length
    values = {
        "accel_h": [],
        "accel_tan": [],
        "accel_norm": [],
        "accel_vertical": [],
        "jerk_h": [],
    }
    for sample in samples:
        acceleration = sample["a"]
        jerk = sample["j"]
        if all(finite(value) for value in acceleration):
            accel_h = math.hypot(acceleration[0], acceleration[1])
            if accel_h <= 50.0:
                values["accel_h"].append(accel_h)
                values["accel_tan"].append(abs(acceleration[0] * unit_n + acceleration[1] * unit_e))
                values["accel_norm"].append(abs(unit_n * acceleration[1] - unit_e * acceleration[0]))
                values["accel_vertical"].append(abs(acceleration[2]))
        if all(finite(value) for value in jerk):
            jerk_h = math.hypot(jerk[0], jerk[1])
            if jerk_h <= 500.0:
                values["jerk_h"].append(jerk_h)
    return values


def fraction_above(values, threshold):
    values = [value for value in values if finite(value)]
    return sum(value > threshold for value in values) / len(values) if values else NAN


def segment_metrics(segment, rows, actual_kinematics, command_kinematics, command_stream, bag_start, args):
    full_rows = [row for row in rows if row["segment_seq"] == segment.sequence]
    steady_rows = [row for row in full_rows if row["steady"]]
    quality_rows = steady_rows if len(steady_rows) >= 5 else []
    quality_duration = (
        quality_rows[-1]["t_abs"] - quality_rows[0]["t_abs"]
        if len(quality_rows) >= 2
        else 0.0
    )
    xte_full = [abs(row["xte_m"]) for row in full_rows if finite(row["xte_m"])]
    xte_steady = [abs(row["xte_m"]) for row in quality_rows if finite(row["xte_m"])]
    xte_signed = [row["xte_m"] for row in quality_rows if finite(row["xte_m"])]
    full_xte_stats = value_stats(xte_full)
    xte_stats = value_stats(xte_steady)

    _, xte_crossings_per_10s, xte_period = reversal_metrics(
        [(row["t_abs"], row["xte_m"]) for row in quality_rows],
        args.xte_hysteresis,
        min_interval=0.25,
    )

    yaw_error_desired = [
        abs(degrees(row["yaw_error_desired"]))
        for row in quality_rows
        if finite(row["yaw_error_desired"])
    ]
    yaw_error_segment = [
        abs(degrees(row["yaw_error_segment"]))
        for row in quality_rows
        if finite(row["yaw_error_segment"])
    ]
    desired_yaw_rate = [
        abs(degrees(row["desired_yaw_rate"]))
        for row in quality_rows
        if finite(row["desired_yaw_rate"])
    ]
    command_yaw_rate = [
        abs(degrees(row["cmd_yaw_rate"]))
        for row in quality_rows
        if finite(row["cmd_yaw_rate"])
    ]
    actual_yaw_rate = [
        abs(degrees(row["actual_yaw_rate"]))
        for row in quality_rows
        if finite(row["actual_yaw_rate"])
    ]
    yaw_command_error = [
        abs(degrees(row["cmd_yaw_rate"] - row["expected_yaw_rate"]))
        for row in quality_rows
        if finite(row["cmd_yaw_rate"]) and finite(row["expected_yaw_rate"])
    ]
    yaw_reversal_count, yaw_reversals_per_10s, yaw_period = reversal_metrics(
        [(row["t_abs"], row["actual_yaw_rate"]) for row in quality_rows],
        math.radians(args.yaw_rate_reversal_threshold_deg_s),
        min_interval=0.25,
    )
    saturation_samples = [
        row
        for row in quality_rows
        if finite(row["cmd_yaw_rate"]) and finite(row["cmd_speed_h"])
        and row["cmd_speed_h"] > args.yaw_speed_gate
    ]
    saturation_fraction = (
        sum(abs(row["cmd_yaw_rate"]) >= 0.98 * args.yaw_max_rate for row in saturation_samples)
        / len(saturation_samples)
        if saturation_samples
        else NAN
    )
    speed_errors = [
        math.sqrt(
            (row["cmd_vn"] - row["actual_vn"]) ** 2
            + (row["cmd_ve"] - row["actual_ve"]) ** 2
            + (row["cmd_vd"] - row["actual_vd"]) ** 2
        )
        for row in quality_rows
        if all(
            finite(row[name])
            for name in ("cmd_vn", "cmd_ve", "cmd_vd", "actual_vn", "actual_ve", "actual_vd")
        )
    ]

    l1_values = [row["l1_m"] for row in full_rows if finite(row["l1_m"])]
    l1_min_m = min(l1_values) if l1_values else NAN
    l1_max_m = max(l1_values) if l1_values else NAN
    l1_p50_m = statistics.median(l1_values) if l1_values else NAN

    actual_samples = select_kinematics(actual_kinematics, segment, quality_rows)
    command_samples = select_kinematics(command_kinematics, segment, quality_rows)
    actual_motion = decompose_kinematics(actual_samples, segment)
    command_motion = decompose_kinematics(command_samples, segment)
    notch, _, notch_delta = command_notch_at(command_stream, segment.t_end)

    yaw_desired_stats = value_stats(yaw_error_desired)
    yaw_segment_stats = value_stats(yaw_error_segment)
    desired_rate_stats = value_stats(desired_yaw_rate)
    command_rate_stats = value_stats(command_yaw_rate)
    actual_rate_stats = value_stats(actual_yaw_rate)
    yaw_reconstruction_stats = value_stats(yaw_command_error)
    speed_error_stats = value_stats(speed_errors)
    actual_accel_h = value_stats(actual_motion.get("accel_h", []))
    actual_accel_tan = value_stats(actual_motion.get("accel_tan", []))
    actual_accel_norm = value_stats(actual_motion.get("accel_norm", []))
    actual_accel_vertical = value_stats(actual_motion.get("accel_vertical", []))
    actual_jerk_h = value_stats(actual_motion.get("jerk_h", []))
    command_accel_h = value_stats(command_motion.get("accel_h", []))
    command_accel_tan = value_stats(command_motion.get("accel_tan", []))
    command_accel_norm = value_stats(command_motion.get("accel_norm", []))
    command_accel_vertical = value_stats(command_motion.get("accel_vertical", []))
    command_jerk_h = value_stats(command_motion.get("jerk_h", []))

    flags = []
    if segment.p0 is None or segment.p1 is None:
        flags.append("GEOMETRY_UNKNOWN")
    if finite(segment.p1_spread) and segment.p1_spread > 0.2:
        flags.append("TARGET_CHANGED")
    if len(steady_rows) < 5:
        flags.append("NO_STEADY_WINDOW")
    if finite(xte_stats["p95"]) and xte_stats["p95"] > args.xte_p95_limit:
        flags.append("XTE")
    if finite(yaw_desired_stats["p95"]) and yaw_desired_stats["p95"] > args.yaw_error_p95_limit_deg:
        flags.append("YAW_ERROR")
    if (
        quality_duration >= 3.0
        and yaw_reversal_count >= 2
        and finite(yaw_reversals_per_10s)
        and yaw_reversals_per_10s > args.yaw_reversal_limit
    ):
        flags.append("YAW_WAG")
    if finite(actual_accel_h["p95"]) and actual_accel_h["p95"] > args.accel_p95_limit:
        flags.append("ACCEL")
    if finite(actual_jerk_h["p95"]) and actual_jerk_h["p95"] > args.jerk_p95_limit:
        flags.append("JERK")
    if notch:
        flags.append("CMD_ZERO_NOTCH")

    diagnosis = ""
    if "XTE" in flags and finite(desired_rate_stats["p95"]) and desired_rate_stats["p95"] > 10.0:
        diagnosis = "L1期望航向自身变化快，优先检查横向回路/lookahead"
    elif "YAW_ERROR" in flags and finite(desired_rate_stats["p95"]) and desired_rate_stats["p95"] <= 10.0:
        diagnosis = "期望航向较稳但机头跟踪差，优先检查yaw回路/PX4响应"
    elif "CMD_ZERO_NOTCH" in flags:
        diagnosis = "航点切换存在单帧零速指令"
    elif "ACCEL" in flags or "JERK" in flags:
        if finite(command_accel_h["p95"]) and command_accel_h["p95"] > args.accel_p95_limit:
            diagnosis = "上层速度指令不平滑"
        else:
            diagnosis = "上层指令较平滑，优先检查PX4速度环/实机响应"

    length = NAN
    if segment.p0 is not None and segment.p1 is not None:
        length = math.hypot(segment.p1[0] - segment.p0[0], segment.p1[1] - segment.p0[1])

    return {
        "segment_seq": segment.sequence,
        "label": segment.label,
        "skill_id": segment.skill_id,
        "skill_type": segment.skill_type,
        "phase": segment.phase,
        "index": segment.index,
        "time_start_s": segment.t_start - bag_start,
        "time_end_s": segment.t_end - bag_start,
        "duration_s": segment.t_end - segment.t_start,
        "p0_n": segment.p0[0] if segment.p0 else NAN,
        "p0_e": segment.p0[1] if segment.p0 else NAN,
        "p1_n": segment.p1[0] if segment.p1 else NAN,
        "p1_e": segment.p1[1] if segment.p1 else NAN,
        "p0_source": segment.p0_source,
        "p1_spread_m": segment.p1_spread,
        "segment_length_m": length,
        "speed_setpoint_mps": segment.speed_setpoint,
        "samples_full": len(full_rows),
        "samples_steady": len(steady_rows),
        "xte_full_median_abs_m": full_xte_stats["median"],
        "xte_full_p95_abs_m": full_xte_stats["p95"],
        "xte_steady_median_abs_m": xte_stats["median"],
        "xte_steady_rms_m": xte_stats["rms"],
        "xte_steady_p95_abs_m": xte_stats["p95"],
        "xte_steady_max_abs_m": xte_stats["max"],
        "xte_steady_signed_median_m": statistics.median(xte_signed) if xte_signed else NAN,
        "xte_gt_0p25_fraction": fraction_above(xte_steady, 0.25),
        "xte_gt_0p5_fraction": fraction_above(xte_steady, 0.5),
        "xte_gt_1p0_fraction": fraction_above(xte_steady, 1.0),
        "xte_crossings_per_10s": xte_crossings_per_10s,
        "xte_estimated_period_s": xte_period,
        "yaw_error_desired_median_deg": yaw_desired_stats["median"],
        "yaw_error_desired_p95_deg": yaw_desired_stats["p95"],
        "yaw_error_desired_max_deg": yaw_desired_stats["max"],
        "yaw_error_segment_median_deg": yaw_segment_stats["median"],
        "yaw_error_segment_p95_deg": yaw_segment_stats["p95"],
        "desired_yaw_rate_p95_deg_s": desired_rate_stats["p95"],
        "cmd_yaw_rate_p95_deg_s": command_rate_stats["p95"],
        "actual_yaw_rate_p95_deg_s": actual_rate_stats["p95"],
        "yaw_reversals_per_10s": yaw_reversals_per_10s,
        "yaw_estimated_period_s": yaw_period,
        "yaw_rate_saturation_fraction": saturation_fraction,
        "yaw_command_reconstruction_error_p95_deg_s": yaw_reconstruction_stats["p95"],
        "speed_tracking_error_p95_mps": speed_error_stats["p95"],
        "actual_accel_h_median_mps2": actual_accel_h["median"],
        "actual_accel_h_p95_mps2": actual_accel_h["p95"],
        "actual_accel_h_max_mps2": actual_accel_h["max"],
        "actual_accel_tan_p95_mps2": actual_accel_tan["p95"],
        "actual_accel_norm_p95_mps2": actual_accel_norm["p95"],
        "actual_accel_vertical_p95_mps2": actual_accel_vertical["p95"],
        "actual_jerk_h_p95_mps3": actual_jerk_h["p95"],
        "actual_jerk_h_max_mps3": actual_jerk_h["max"],
        "cmd_accel_h_median_mps2": command_accel_h["median"],
        "cmd_accel_h_p95_mps2": command_accel_h["p95"],
        "cmd_accel_h_max_mps2": command_accel_h["max"],
        "cmd_accel_tan_p95_mps2": command_accel_tan["p95"],
        "cmd_accel_norm_p95_mps2": command_accel_norm["p95"],
        "cmd_accel_vertical_p95_mps2": command_accel_vertical["p95"],
        "cmd_jerk_h_p95_mps3": command_jerk_h["p95"],
        "cmd_jerk_h_max_mps3": command_jerk_h["max"],
        "command_zero_notch": notch,
        "command_zero_notch_delta_v_mps": notch_delta,
        "l1_min_m": l1_min_m,
        "l1_p50_m": l1_p50_m,
        "l1_max_m": l1_max_m,
        "flags": "|".join(flags),
        "diagnosis": diagnosis,
    }


SAMPLE_FIELDS = [
    "time_s",
    "segment_seq",
    "skill_id",
    "skill_type",
    "phase",
    "phase_name",
    "index",
    "label",
    "steady",
    "terminal",
    "avoidance_active",
    "pose_n",
    "pose_e",
    "pose_d",
    "xte_m",
    "along_m",
    "distance_to_target_m",
    "segment_length_m",
    "l1_m",
    "segment_bearing_deg",
    "desired_yaw_deg",
    "actual_yaw_deg",
    "yaw_error_desired_deg",
    "yaw_error_segment_deg",
    "desired_yaw_rate_deg_s",
    "cmd_yaw_rate_deg_s",
    "expected_yaw_rate_deg_s",
    "actual_yaw_rate_deg_s",
    "pose_yaw_rate_deg_s",
    "actual_vn",
    "actual_ve",
    "actual_vd",
    "actual_speed_h",
    "cmd_vn",
    "cmd_ve",
    "cmd_vd",
    "cmd_speed_h",
    "course_deg",
]


SEGMENT_FIELDS = [
    "segment_seq",
    "label",
    "skill_id",
    "skill_type",
    "phase",
    "index",
    "time_start_s",
    "time_end_s",
    "duration_s",
    "p0_n",
    "p0_e",
    "p1_n",
    "p1_e",
    "p0_source",
    "p1_spread_m",
    "segment_length_m",
    "speed_setpoint_mps",
    "samples_full",
    "samples_steady",
    "xte_full_median_abs_m",
    "xte_full_p95_abs_m",
    "xte_steady_median_abs_m",
    "xte_steady_rms_m",
    "xte_steady_p95_abs_m",
    "xte_steady_max_abs_m",
    "xte_steady_signed_median_m",
    "xte_gt_0p25_fraction",
    "xte_gt_0p5_fraction",
    "xte_gt_1p0_fraction",
    "xte_crossings_per_10s",
    "xte_estimated_period_s",
    "yaw_error_desired_median_deg",
    "yaw_error_desired_p95_deg",
    "yaw_error_desired_max_deg",
    "yaw_error_segment_median_deg",
    "yaw_error_segment_p95_deg",
    "desired_yaw_rate_p95_deg_s",
    "cmd_yaw_rate_p95_deg_s",
    "actual_yaw_rate_p95_deg_s",
    "yaw_reversals_per_10s",
    "yaw_estimated_period_s",
    "yaw_rate_saturation_fraction",
    "yaw_command_reconstruction_error_p95_deg_s",
    "speed_tracking_error_p95_mps",
    "actual_accel_h_median_mps2",
    "actual_accel_h_p95_mps2",
    "actual_accel_h_max_mps2",
    "actual_accel_tan_p95_mps2",
    "actual_accel_norm_p95_mps2",
    "actual_accel_vertical_p95_mps2",
    "actual_jerk_h_p95_mps3",
    "actual_jerk_h_max_mps3",
    "cmd_accel_h_median_mps2",
    "cmd_accel_h_p95_mps2",
    "cmd_accel_h_max_mps2",
    "cmd_accel_tan_p95_mps2",
    "cmd_accel_norm_p95_mps2",
    "cmd_accel_vertical_p95_mps2",
    "cmd_jerk_h_p95_mps3",
    "cmd_jerk_h_max_mps3",
    "command_zero_notch",
    "command_zero_notch_delta_v_mps",
    "l1_min_m",
    "l1_p50_m",
    "l1_max_m",
    "flags",
    "diagnosis",
]


def csv_sample_row(row):
    output = {field: row.get(field, "") for field in SAMPLE_FIELDS}
    output.update(
        {
            "segment_bearing_deg": degrees(row["segment_bearing"]),
            "desired_yaw_deg": degrees(row["desired_yaw"]),
            "actual_yaw_deg": degrees(row["actual_yaw"]),
            "yaw_error_desired_deg": degrees(row["yaw_error_desired"]),
            "yaw_error_segment_deg": degrees(row["yaw_error_segment"]),
            "desired_yaw_rate_deg_s": degrees(row["desired_yaw_rate"]),
            "cmd_yaw_rate_deg_s": degrees(row["cmd_yaw_rate"]),
            "expected_yaw_rate_deg_s": degrees(row["expected_yaw_rate"]),
            "actual_yaw_rate_deg_s": degrees(row["actual_yaw_rate"]),
            "pose_yaw_rate_deg_s": degrees(row["pose_yaw_rate"]),
            "course_deg": degrees(row["course"]),
        }
    )
    return output


def write_csv(path, fields, rows):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def parse_replay_specs(specs, args_template):
    """Convert ['1.0:0.8', '1.5:1.0'] into [(label, args_copy), ...]."""
    import copy
    if not specs:
        return []
    out = []
    for spec in specs:
        if ":" not in spec:
            raise ValueError(
                "invalid --replay-gain spec {!r}; expected 'GAIN:PERIOD' (e.g. 1.0:0.8)".format(spec)
            )
        gain_str, period_str = spec.split(":", 1)
        gain = float(gain_str)
        period = float(period_str)
        if gain < 0.0:
            raise ValueError("replay gain must be >= 0, got {}".format(gain))
        if period < 0.0:
            raise ValueError("replay l1_min_period must be >= 0, got {}".format(period))
        replay_args = copy.copy(args_template)
        replay_args.xte_damping_gain = gain
        replay_args.l1_min_period = period
        label = "gain={:g}_period={:g}".format(gain, period)
        out.append((label, replay_args))
    return out


REPLAY_METRIC_KEYS = [
    ("xte_steady_p95_abs_m", "XTE P95 (m)"),
    ("xte_steady_rms_m", "XTE RMS (m)"),
    ("xte_steady_max_abs_m", "XTE max (m)"),
    ("yaw_error_desired_p95_deg", "yaw err P95 (deg)"),
    ("yaw_error_desired_max_deg", "yaw err max (deg)"),
    ("desired_yaw_rate_p95_deg_s", "des yaw-rate P95 (deg/s)"),
    ("actual_yaw_rate_p95_deg_s", "act yaw-rate P95 (deg/s)"),
    ("yaw_reversals_per_10s", "yaw rev/10s"),
    ("yaw_estimated_period_s", "yaw period (s)"),
    ("l1_p50_m", "L1 P50 (m)"),
    ("actual_accel_h_p95_mps2", "accel P95 (m/s^2)"),
    ("flags", "flags"),
]


def write_replay_csv(path, segments, replay_metrics_by_run):
    """Write one row per segment with one column per (variant × metric)."""
    run_labels = ["baseline"] + [
        label for label in replay_metrics_by_run.keys() if label != "baseline"
    ]
    fields = [
        "segment_seq",
        "label",
        "segment_length_m",
        "samples_steady",
        "skill_id",
    ]
    for run_label in run_labels:
        for key, _ in REPLAY_METRIC_KEYS:
            fields.append("{}__{}".format(run_label, key))
    rows_out = []
    for idx, segment in enumerate(segments):
        segment_length = NAN
        if segment.p0 is not None and segment.p1 is not None:
            segment_length = math.hypot(
                segment.p1[0] - segment.p0[0],
                segment.p1[1] - segment.p0[1],
            )
        row = {
            "segment_seq": segment.sequence,
            "label": segment.label,
            "segment_length_m": segment_length,
            "skill_id": segment.skill_id,
        }
        baseline_metrics = replay_metrics_by_run.get("baseline")
        if baseline_metrics is not None:
            row["samples_steady"] = baseline_metrics[idx]["samples_steady"]
        for run_label in run_labels:
            metrics_list = replay_metrics_by_run.get(run_label)
            if metrics_list is None:
                continue
            metric = metrics_list[idx]
            for key, _ in REPLAY_METRIC_KEYS:
                value = metric.get(key, "")
                row["{}__{}".format(run_label, key)] = value
        rows_out.append(row)
    write_csv(path, fields, rows_out)


def build_replay_summary(segments, replay_metrics_by_run):
    """Build a textual summary comparing replay variants to baseline."""
    lines = ["=== 参数预演对比 (--replay-gain) ==="]
    run_labels = ["baseline"] + [
        label for label in replay_metrics_by_run.keys() if label != "baseline"
    ]
    if len(run_labels) < 2:
        lines.append("无 replay 变量。")
        return "\n".join(lines) + "\n"

    baseline_metrics = replay_metrics_by_run.get("baseline", [])

    def safe_get(metrics, idx, key):
        if idx >= len(metrics):
            return NAN
        return metrics[idx].get(key, NAN)

    def fmt(value):
        return "{:.3f}".format(value) if finite(value) else "-"

    lines.append("baseline = 当前 controller (gain=0, l1_min_period=0,L1_min={:.1f}m,L1_period={:.2f}s)".format(
        3.0, 0.6))
    lines.append("")

    lines.append("--- 关键 flag 计数 (跨航段汇总) ---")
    header = "{:>32s}".format("variant")
    header += "  " + "  ".join(
        "{:>6s}".format(flag) for flag in
        ("XTE", "YAW_WAG", "YAW_ERROR", "ACCEL", "CMD_ZERO_NOTCH", "NO_STEADY_WINDOW", "total")
    )
    lines.append(header)
    for run_label in run_labels:
        metrics_list = replay_metrics_by_run.get(run_label, [])
        flag_counter = {"XTE": 0, "YAW_WAG": 0, "YAW_ERROR": 0, "ACCEL": 0, "CMD_ZERO_NOTCH": 0, "NO_STEADY_WINDOW": 0}
        for metric in metrics_list:
            flags_str = metric.get("flags") or ""
            for flag in flag_counter:
                if flag in flags_str:
                    flag_counter[flag] += 1
        any_flag_total = sum(1 for m in metrics_list if (m.get("flags") or ""))
        line = "{:>32s}".format(run_label)
        line += "  " + "  ".join(
            "{:>6d}".format(flag_counter[flag]) for flag in
            ("XTE", "YAW_WAG", "YAW_ERROR", "ACCEL", "CMD_ZERO_NOTCH", "NO_STEADY_WINDOW")
        )
        line += "  {:>6d}".format(any_flag_total)
        lines.append(line)
    lines.append("")

    # Per-segment focus: only segments where baseline showed problems
    focus_rows = []
    for idx, segment in enumerate(segments):
        xte_p95 = safe_get(baseline_metrics, idx, "xte_steady_p95_abs_m")
        yaw_err_p95 = safe_get(baseline_metrics, idx, "yaw_error_desired_p95_deg")
        revs = safe_get(baseline_metrics, idx, "yaw_reversals_per_10s")
        period = safe_get(baseline_metrics, idx, "yaw_estimated_period_s")
        wag_like = finite(revs) and finite(period) and revs >= 2.0 and 3.0 <= period <= 12.0
        if (
            (finite(xte_p95) and xte_p95 >= 0.5)
            or (finite(yaw_err_p95) and yaw_err_p95 >= 8.0)
            or wag_like
        ):
            focus_rows.append((idx, segment))
    if not focus_rows:
        lines.append("--- 重点航段 ---")
        lines.append("baseline 没有命中 XTE P95>=0.5m / yaw err P95>=8° / wag-like 摆动的航段,无需对比。")
        lines.append("")
    else:
        lines.append("--- 重点航段对比 (baseline 命中问题) ---")
        # Column header
        header = "{:>5s}  {:<22s}".format("seq", "label")
        for run_label in run_labels:
            header += "  {:>16s}".format(run_label[:16])
        lines.append(header)
        for idx, segment in focus_rows:
            seq = segment.sequence
            label = segment.label
            for key, pretty in REPLAY_METRIC_KEYS[:9]:
                row_line = "{:>5d}  {:<22s}".format(seq, label)
                for run_label in run_labels:
                    value = safe_get(replay_metrics_by_run.get(run_label, []), idx, key)
                    if key == "flags":
                        cell = value if value else "-"
                    else:
                        cell = fmt(value)
                    row_line += "  {:>16s}".format(cell[:16])
                row_line += "  " + pretty
                lines.append(row_line)
            # Flag row
            flag_line = "{:>5s}  {:<22s}".format("", "")
            for run_label in run_labels:
                flags_str = safe_get(replay_metrics_by_run.get(run_label, []), idx, "flags")
                cell = flags_str if flags_str else "-"
                flag_line += "  {:>16s}".format(cell[:16])
            flag_line += "  flags"
            lines.append(flag_line)
            lines.append("")
    return "\n".join(lines) + "\n"


def build_summary(infos, namespace, topics, streams, segments, rows, metrics, bag_start, bag_end):
    lines = [
        "=== 航迹质量 bag 分析 ===",
        "UAV: /{}".format(namespace),
        "bag 时长: {:.2f} s, 航点执行采样: {}, 航段: {}".format(bag_end - bag_start, len(rows), len(segments)),
        "bags:",
    ]
    lines.extend("  - {}".format(info.path) for info in infos)
    missing_optional = [
        key
        for key in ("velocity", "command", "imu", "target", "skill", "control", "avoidance")
        if not streams[key]
    ]
    if missing_optional:
        lines.append("缺失可选话题: {}".format(", ".join(topics[key] for key in missing_optional)))
    lines.append("")
    lines.append("=== 逐航段 ===")

    for metric in metrics:
        lines.append(
            "[{segment_seq:02d}] {label} skill={skill_id} t={time_start_s:.2f}..{time_end_s:.2f}s "
            "line=({p0_n_fmt},{p0_e_fmt})->({p1_n_fmt},{p1_e_fmt}) steady={samples_steady}".format(
                p0_n_fmt=format_number(metric["p0_n"]),
                p0_e_fmt=format_number(metric["p0_e"]),
                p1_n_fmt=format_number(metric["p1_n"]),
                p1_e_fmt=format_number(metric["p1_e"]),
                **metric
            )
        )
        lines.append(
            "  压线 |XTE| median/P95/max = {}/{}/{} m, 过线反转={}/10s, 周期={} s".format(
                format_number(metric["xte_steady_median_abs_m"]),
                format_number(metric["xte_steady_p95_abs_m"]),
                format_number(metric["xte_steady_max_abs_m"]),
                format_number(metric["xte_crossings_per_10s"]),
                format_number(metric["xte_estimated_period_s"]),
            )
        )
        lines.append(
            "  航向 |yaw误差| P95={} deg, desired/actual yaw-rate P95={}/{} deg/s, 左右反转={}/10s, 饱和={}".format(
                format_number(metric["yaw_error_desired_p95_deg"]),
                format_number(metric["desired_yaw_rate_p95_deg_s"]),
                format_number(metric["actual_yaw_rate_p95_deg_s"]),
                format_number(metric["yaw_reversals_per_10s"]),
                format_number(metric["yaw_rate_saturation_fraction"]),
            )
        )
        lines.append(
            "  平顺 actual accel/jerk P95={}/{} m/s²,m/s³; cmd accel/jerk P95={}/{}; 零速脉冲={}".format(
                format_number(metric["actual_accel_h_p95_mps2"]),
                format_number(metric["actual_jerk_h_p95_mps3"]),
                format_number(metric["cmd_accel_h_p95_mps2"]),
                format_number(metric["cmd_jerk_h_p95_mps3"]),
                "是" if metric["command_zero_notch"] else "否",
            )
        )
        if metric["flags"]:
            lines.append("  FLAGS: {}{}".format(metric["flags"], " — " + metric["diagnosis"] if metric["diagnosis"] else ""))

    lines.append("")
    lines.append("=== 汇总判断 ===")
    valid_metrics = [metric for metric in metrics if metric["samples_steady"] >= 5]
    if not valid_metrics:
        lines.append("没有足够的稳定直线区样本；请确保从任务开始前录包并包含完整 waypoint 段。")
        return "\n".join(lines) + "\n"

    worst_xte = max(valid_metrics, key=lambda item: item["xte_steady_p95_abs_m"] if finite(item["xte_steady_p95_abs_m"]) else -1.0)
    worst_yaw = max(valid_metrics, key=lambda item: item["yaw_error_desired_p95_deg"] if finite(item["yaw_error_desired_p95_deg"]) else -1.0)
    worst_accel = max(valid_metrics, key=lambda item: item["actual_accel_h_p95_mps2"] if finite(item["actual_accel_h_p95_mps2"]) else -1.0)
    lines.append(
        "最差压线: {} P95={} m；最差航向: {} yaw误差P95={} deg；最差平顺性: {} accel P95={} m/s²。".format(
            worst_xte["label"],
            format_number(worst_xte["xte_steady_p95_abs_m"]),
            worst_yaw["label"],
            format_number(worst_yaw["yaw_error_desired_p95_deg"]),
            worst_accel["label"],
            format_number(worst_accel["actual_accel_h_p95_mps2"]),
        )
    )
    diagnoses = []
    if any(metric["command_zero_notch"] for metric in metrics):
        diagnoses.append("检测到航点切换单帧零速，可能直接制造加速度/jerk尖峰")
    if any("XTE" in metric["flags"] and metric["desired_yaw_rate_p95_deg_s"] > 10.0 for metric in valid_metrics):
        diagnoses.append("XTE与期望航向同时摆动，优先检查L1/lookahead横向回路")
    if any("YAW_ERROR" in metric["flags"] and metric["desired_yaw_rate_p95_deg_s"] <= 10.0 for metric in valid_metrics):
        diagnoses.append("期望航向较稳但机头误差大，优先检查yaw增益、死区和PX4 yaw-rate响应")
    if any("ACCEL" in metric["flags"] or "JERK" in metric["flags"] for metric in valid_metrics):
        diagnoses.append("存在实际加速度/jerk超限，需对照cmd指标区分上层指令与PX4响应")
    if diagnoses:
        lines.extend("- {}".format(item) for item in diagnoses)
    else:
        lines.append("当前阈值下未发现明显异常；仍应结合 overview 图检查细小周期振荡。")
    return "\n".join(lines) + "\n"


def plot_overview(path, rows, segments, actual_kinematics, command_kinematics, bag_start):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise ValueError("--plot requires matplotlib: {}".format(exc))

    if not rows:
        raise ValueError("cannot plot: no waypoint execution samples")

    figure, axes = plt.subplots(3, 2, figsize=(16, 13), sharex=False)
    axes = axes.flatten()

    axes[0].plot([row["pose_e"] for row in rows], [row["pose_n"] for row in rows], linewidth=1.0, label="actual")
    for segment in segments:
        if segment.p0 is not None and segment.p1 is not None:
            axes[0].plot([segment.p0[1], segment.p1[1]], [segment.p0[0], segment.p1[0]], "--", linewidth=1.0)
            axes[0].text(segment.p1[1], segment.p1[0], segment.label, fontsize=7)
    axes[0].set_title("Horizontal track (NED)")
    axes[0].set_xlabel("East [m]")
    axes[0].set_ylabel("North [m]")
    axes[0].axis("equal")
    axes[0].grid(True)

    times = [row["time_s"] for row in rows]
    axes[1].plot(times, [row["xte_m"] for row in rows], linewidth=0.9)
    axes[1].axhline(0.0, color="black", linewidth=0.6)
    axes[1].set_title("Signed cross-track error")
    axes[1].set_ylabel("XTE [m]")
    axes[1].grid(True)

    actual_yaw = unwrap_angles([row["actual_yaw"] for row in rows])
    desired_yaw = unwrap_angles([row["desired_yaw"] for row in rows])
    axes[2].plot(times, [degrees(value) for value in actual_yaw], label="actual", linewidth=0.9)
    axes[2].plot(times, [degrees(value) for value in desired_yaw], label="desired", linewidth=0.9)
    axes[2].set_title("Yaw")
    axes[2].set_ylabel("unwrapped yaw [deg]")
    axes[2].legend()
    axes[2].grid(True)

    axes[3].plot(times, [degrees(row["actual_yaw_rate"]) for row in rows], label="actual", linewidth=0.8)
    axes[3].plot(times, [degrees(row["cmd_yaw_rate"]) for row in rows], label="command", linewidth=0.8)
    axes[3].plot(times, [degrees(row["desired_yaw_rate"]) for row in rows], label="desired bearing rate", linewidth=0.8)
    axes[3].set_title("Yaw rate")
    axes[3].set_ylabel("deg/s")
    axes[3].legend()
    axes[3].grid(True)

    axes[4].plot(times, [row["actual_speed_h"] for row in rows], label="actual", linewidth=0.9)
    axes[4].plot(times, [row["cmd_speed_h"] for row in rows], label="command", linewidth=0.9)
    axes[4].set_title("Horizontal speed")
    axes[4].set_ylabel("m/s")
    axes[4].set_xlabel("bag-relative time [s]")
    axes[4].legend()
    axes[4].grid(True)

    active_times = [row["t_abs"] for row in rows]

    def during_waypoint_execution(sample):
        index = bisect_right(active_times, sample["t"])
        neighbors = []
        if index > 0:
            neighbors.append(active_times[index - 1])
        if index < len(active_times):
            neighbors.append(active_times[index])
        return bool(neighbors) and min(abs(sample["t"] - value) for value in neighbors) <= 0.2

    plotted_actual = [sample for sample in actual_kinematics if during_waypoint_execution(sample)]
    plotted_command = [sample for sample in command_kinematics if during_waypoint_execution(sample)]
    actual_time = [sample["t"] - bag_start for sample in plotted_actual]
    command_time = [sample["t"] - bag_start for sample in plotted_command]
    axes[5].plot(actual_time, [horizontal_norm(sample["a"]) for sample in plotted_actual], label="actual accel", linewidth=0.8)
    axes[5].plot(command_time, [horizontal_norm(sample["a"]) for sample in plotted_command], label="cmd accel", linewidth=0.7)
    axes[5].plot(actual_time, [horizontal_norm(sample["j"]) for sample in plotted_actual], label="actual jerk", linewidth=0.6, alpha=0.7)
    axes[5].set_title("Horizontal acceleration / jerk")
    axes[5].set_ylabel("m/s^2 or m/s^3")
    axes[5].set_xlabel("bag-relative time [s]")
    axes[5].legend()
    axes[5].grid(True)

    for axis in axes[1:]:
        for segment in segments:
            axis.axvline(segment.t_start - bag_start, color="gray", linewidth=0.4, alpha=0.5)
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def main():
    args = parse_args()
    try:
        infos = inspect_bags(args.bags)
        namespace = discover_namespace(infos, args.uav)
        topics = topic_map(namespace)
        streams = load_streams(infos, topics)
    except ValueError as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 2

    required_missing = [key for key in ("pose", "status", "target") if not streams[key]]
    if required_missing:
        print(
            "error: missing required topics: {}".format(", ".join(topics[key] for key in required_missing)),
            file=sys.stderr,
        )
        return 2

    bag_start = min(info.start for info in infos)
    bag_end = max(info.end for info in infos)
    segments = reconstruct_segments(streams, bag_start, bag_end)
    if not segments:
        print("error: no active ARRIVE/SKILL_AREA waypoint segments found", file=sys.stderr)
        return 2

    rows = build_sample_rows(streams, segments, bag_start, args)
    actual_kinematics = robust_kinematics(streams["velocity"])
    command_kinematics = robust_kinematics(streams["command"])
    metrics = [
        segment_metrics(
            segment,
            rows,
            actual_kinematics,
            command_kinematics,
            streams["command"],
            bag_start,
            args,
        )
        for segment in segments
    ]

    output_dir = os.path.abspath(args.output_dir or os.path.dirname(infos[0].path) or ".")
    prefix = args.prefix or os.path.splitext(os.path.basename(infos[0].path))[0]

    # ===== 参数预演 (--replay-gain) =====
    # 用同一份 bag 和同一组航段,只改变 args.xte_damping_gain 和 args.l1_min_period
    # 重算 desired_yaw + 下游 yaw 指标,然后输出对比 CSV。
    replay_specs = parse_replay_specs(args.replay_gain, args)
    replay_runs = [("baseline", args)]
    for label, replay_args in replay_specs:
        replay_runs.append((label, replay_args))

    replay_metrics_by_run = {}
    if len(replay_runs) > 1:
        for label, run_args in replay_runs:
            run_rows = build_sample_rows(streams, segments, bag_start, run_args)
            run_metrics = [
                segment_metrics(
                    segment,
                    run_rows,
                    actual_kinematics,
                    command_kinematics,
                    streams["command"],
                    bag_start,
                    run_args,
                )
                for segment in segments
            ]
            replay_metrics_by_run[label] = run_metrics
    try:
        os.makedirs(output_dir, exist_ok=True)
        samples_path = os.path.join(output_dir, prefix + ".track_samples.csv")
        segments_path = os.path.join(output_dir, prefix + ".track_segments.csv")
        summary_path = os.path.join(output_dir, prefix + ".track_summary.txt")
        write_csv(samples_path, SAMPLE_FIELDS, [csv_sample_row(row) for row in rows])
        write_csv(segments_path, SEGMENT_FIELDS, metrics)
        if replay_metrics_by_run:
            replay_path = os.path.join(output_dir, prefix + ".track_replay.csv")
            replay_text_path = os.path.join(output_dir, prefix + ".track_replay.txt")
            write_replay_csv(replay_path, segments, replay_metrics_by_run)
            replay_text = build_replay_summary(segments, replay_metrics_by_run)
            with open(replay_text_path, "w", encoding="utf-8") as handle:
                handle.write(replay_text)
        summary = build_summary(
            infos,
            namespace,
            topics,
            streams,
            segments,
            rows,
            metrics,
            bag_start,
            bag_end,
        )
        with open(summary_path, "w", encoding="utf-8") as handle:
            handle.write(summary)
        plot_path = None
        if args.plot:
            plot_path = os.path.join(output_dir, prefix + ".track_overview.png")
            plot_overview(plot_path, rows, segments, actual_kinematics, command_kinematics, bag_start)
    except (OSError, ValueError) as exc:
        print("error: cannot write analysis output: {}".format(exc), file=sys.stderr)
        return 2

    print(summary, end="")
    print("输出:")
    print("  - {}".format(samples_path))
    print("  - {}".format(segments_path))
    print("  - {}".format(summary_path))
    if replay_metrics_by_run:
        print("  - {}".format(replay_path))
        print("  - {}".format(replay_text_path))
        print("")
        print(replay_text, end="")
    if plot_path:
        print("  - {}".format(plot_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
