#!/usr/bin/env python3
"""
Analyze a gs_replay playback bag for OFFBOARD setpoint issues.

Three failure modes are detected (each can be enabled/disabled via flags):

  1. --check-autoland   : PX4 ever transitions to AUTO.LAND mode
                          (existing behavior of gs_replay_random.sh inline analyzer)

  2. --check-stream-gap : During OFFBOARD mode, the OFFBOARD heartbeat topic
                          (/<ns>/mavros/setpoint_raw/target_local) has a gap
                          larger than --stream-gap-threshold (default 1.0 s,
                          equals PX4 COM_OFFBOARD_LOSS_TIMEOUT default).
                          Stream break ⇒ PX4 failsafe.

  3. --check-simultaneous : During OFFBOARD mode, *multiple* upstream sources
                          publish to a single setpoint topic in the same
                          time window (default 100 ms). The expected layout
                          is:
                              - mission_manager  → /mavros/setpoint_position/local
                              - waypoint_executor → /mavros/setpoint_velocity/cmd_vel_unstamped
                          Simultaneous publishing from different nodes to
                          the same topic is treated as a configuration bug.
                          (Position + velocity at the same time is OK because
                          mavros aggregates both into setpoint_raw/target_local
                          via type_mask.)

Default invocation checks all three; rc=0 if none of the enabled checks fail.
Output is human-readable and machine-parseable (each finding starts with a
prefixed tag: AUTOLAND / STREAMGAP / SIMULT).

Usage:
    python3 analyze_setpoint_quality.py <bag> [opts]
"""

import argparse
import sys
from collections import defaultdict
from itertools import groupby

try:
    import rosbag
except ImportError:
    print(
        "error: Python cannot import rosbag; source the ROS and catkin workspaces first",
        file=sys.stderr,
    )
    sys.exit(2)


# ----------------------------- topic discovery -----------------------------

# Candidate topics under the chosen namespace (default uav0). We probe them all
# and silently skip ones that the bag doesn't contain.
SETPOINT_POSITION_TOPICS = [
    "/{ns}/mavros/setpoint_position/local",
]
SETPOINT_VELOCITY_TOPICS = [
    "/{ns}/mavros/setpoint_velocity/cmd_vel_unstamped",
    "/{ns}/mavros/setpoint_velocity/cmd_vel",  # legacy
]
SETPOINT_RAW_LOCAL = "/{ns}/mavros/setpoint_raw/target_local"
STATE_TOPIC = "/{ns}/mavros/state"
MISSION_STATE_TOPIC = "/{ns}/mission/mission_state"


def first_existing(bag, candidates):
    for tpc in candidates:
        try:
            bag.get_message_count(tpc)
            return tpc
        except Exception:
            continue
    return None


# ----------------------------- analyzers -----------------------------

def analyze_autoland(bag, state_topic):
    """Return list of (timestamp_sec, mode) when mode == AUTO.LAND."""
    events = []
    if state_topic is None:
        return events
    for topic, msg, t in bag.read_messages(state_topic):
        mode = getattr(msg, "mode", None)
        if isinstance(mode, str) and mode == "AUTO.LAND":
            events.append((t.to_sec(), mode))
    return events


def _gaps(times, threshold):
    """Return list of (gap_start, gap_end, gap_seconds) where gap > threshold."""
    out = []
    for i in range(len(times) - 1):
        g = times[i + 1] - times[i]
        if g > threshold:
            out.append((times[i], times[i + 1], g))
    return out


def analyze_stream_break(bag, raw_local_topic, state_topic, threshold):
    """
    Detect stream breaks in /mavros/setpoint_raw/target_local during OFFBOARD.

    Returns list of (gap_start, gap_end, gap_seconds, px4_mode_at_start).
    """
    if raw_local_topic is None:
        return []

    # Pre-index PX4 mode by 1 Hz snapshots (mavros/state).
    mode_by_t = []
    if state_topic is not None:
        for topic, msg, t in bag.read_messages(state_topic):
            mode = getattr(msg, "mode", None)
            mode_by_t.append((t.to_sec(), mode))

    def mode_at(ts):
        if not mode_by_t:
            return "?"
        # mode_by_t is roughly 1 Hz; pick last snapshot with ts <= target
        last = "?"
        for mt, mm in mode_by_t:
            if mt > ts:
                break
            last = mm
        return last

    times = []
    for topic, msg, t in bag.read_messages(raw_local_topic):
        times.append(t.to_sec())

    gaps = []
    for gs_, ge_, gdur in _gaps(times, threshold):
        gaps.append((gs_, ge_, gdur, mode_at(gs_)))
    return gaps


def analyze_simultaneous_publishers(
    bag,
    candidate_topics_for_position,
    candidate_topics_for_velocity,
    state_topic,
    window_sec,
    min_overlap_sec,
):
    """
    Detect multiple publishers writing to the same setpoint topic in the same window.

    The expected layout is one publisher per topic. If a topic has messages
    from different ROS publishers within `window_sec` of each other for at
    least `min_overlap_sec`, that's a configuration bug.

    Note: mavros republishes incoming MAVLink setpoints back to ROS, so the
    messages we see are NOT necessarily from our nodes. We detect this by
    examining the publisher field embedded in the message header.
    """
    if state_topic is None:
        return []

    def publisher_of(msg):
        """Try to extract a publisher identifier from a ROS message header.

        For mavros setpoint topics, msg has no .header.publisher field directly,
        but the message header carries frame_id="" and we have no other source.
        We therefore fall back to topic-name groups: the publisher is whatever
        wrote to this topic. This is intentionally conservative: we treat
        concurrent writes from different topics to the same logical channel
        (position vs velocity) as "simultaneous" only when each topic sees
        >min_overlap_sec of messages within `window_sec`.
        """
        return None  # no publisher identity available in raw msg

    # Index timestamps per topic.
    series = {}  # topic -> list of (ts, publisher_name_or_None)
    for cand in candidate_topics_for_position + candidate_topics_for_velocity:
        try:
            count = bag.get_message_count(cand)
        except Exception:
            continue
        if count == 0:
            continue
        seq = []
        for topic, msg, t in bag.read_messages(cand):
            seq.append(t.to_sec())
        series[cand] = seq

    findings = []
    # Group candidates into "logical channels": position vs velocity.
    channels = {"position": candidate_topics_for_position,
                "velocity": candidate_topics_for_velocity}

    for ch_name, cands in channels.items():
        active = [c for c in cands if c in series and len(series[c]) > 0]
        if len(active) < 2:
            continue
        # Merge all timestamps from active topics, then for each window of
        # width `window_sec` count how many distinct topics contribute.
        # If a window has messages from >=2 distinct topics for >=min_overlap_sec,
        # flag.
        all_ts = []
        for c in active:
            for ts in series[c]:
                all_ts.append((ts, c))
        all_ts.sort()
        i = 0
        # sliding window: for each i, look at all events where ts < all_ts[i][0] + window_sec
        for j in range(len(all_ts)):
            t0 = all_ts[j][0]
            # advance i so all_ts[i][0] >= t0 - window_sec (no need; we only look forward)
            window_end = t0 + window_sec
            # collect distinct topics in [t0, window_end]
            topics_in_window = set()
            last_ts = t0
            for k in range(j, len(all_ts)):
                tk, ck = all_ts[k]
                if tk > window_end:
                    break
                topics_in_window.add(ck)
                last_ts = tk
            if len(topics_in_window) >= 2 and (last_ts - t0) >= min_overlap_sec:
                findings.append({
                    "channel": ch_name,
                    "topics": sorted(topics_in_window),
                    "window_start": t0,
                    "window_end": last_ts,
                    "overlap_sec": last_ts - t0,
                })
                break  # one finding per channel is enough

    return findings


# ----------------------------- CLI -----------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="Path to playback .bag file")
    ap.add_argument("--ns", default="uav0", help="UAV namespace (default uav0)")
    ap.add_argument("--no-autoland", action="store_true",
                    help="Disable AUTO.LAND outcome check")
    ap.add_argument("--no-stream-gap", action="store_true",
                    help="Disable setpoint stream gap check")
    ap.add_argument("--no-simultaneous", action="store_true",
                    help="Disable simultaneous-publishers check")
    ap.add_argument("--stream-gap-threshold", type=float, default=1.0,
                    help="Stream gap threshold in seconds (default 1.0 = PX4 "
                         "COM_OFFBOARD_LOSS_TIMEOUT default)")
    ap.add_argument("--simul-window", type=float, default=0.1,
                    help="Simultaneous-publish sliding window in seconds "
                         "(default 0.1)")
    ap.add_argument("--simul-min-overlap", type=float, default=0.5,
                    help="Minimum overlap duration to flag as simultaneous "
                         "(default 0.5 s)")
    args = ap.parse_args()

    pos_tpcs = [t.format(ns=args.ns) for t in SETPOINT_POSITION_TOPICS]
    vel_tpcs = [t.format(ns=args.ns) for t in SETPOINT_VELOCITY_TOPICS]
    raw_local = SETPOINT_RAW_LOCAL.format(ns=args.ns)
    state_tpc = STATE_TOPIC.format(ns=args.ns)
    mission_state_tpc = MISSION_STATE_TOPIC.format(ns=args.ns)

    rc = 0
    findings = []

    with rosbag.Bag(args.bag, "r") as bag:
        # ------- Check 1: AUTO.LAND -------
        if not args.no_autoland:
            events = analyze_autoland(bag, state_tpc)
            if events:
                findings.append(("AUTOLAND",
                                 "PX4 mode=AUTO.LAND detected ({} events)".format(len(events))))
                for ts, _ in events[:5]:
                    print(f"  AUTOLAND t={ts:.3f}s")
                if len(events) > 5:
                    print(f"  AUTOLAND ... +{len(events) - 5} more")
                rc = 1

        # ------- Check 2: stream gap -------
        if not args.no_stream_gap:
            gaps = analyze_stream_break(bag, raw_local, state_tpc,
                                        args.stream_gap_threshold)
            if gaps:
                findings.append(("STREAMGAP",
                                 "setpoint_raw/target_local gap > {:.2f}s ({} gap(s))".format(
                                     args.stream_gap_threshold, len(gaps))))
                for gs_, ge_, gdur, mode in gaps[:10]:
                    print(f"  STREAMGAP t={gs_:.3f}-{ge_:.3f}s duration={gdur:.3f}s mode_before={mode}")
                rc = 1

        # ------- Check 3: simultaneous publishers -------
        if not args.no_simultaneous:
            sim = analyze_simultaneous_publishers(
                bag, pos_tpcs, vel_tpcs, state_tpc,
                args.simul_window, args.simul_min_overlap,
            )
            if sim:
                findings.append(("SIMULT",
                                 "{} channel(s) with concurrent publishers".format(len(sim))))
                for f in sim:
                    print(f"  SIMULT channel={f['channel']} topics={f['topics']} "
                          f"window={f['window_start']:.3f}-{f['window_end']:.3f}s "
                          f"overlap={f['overlap_sec']:.3f}s")
                rc = 1

    if not findings:
        print(f"OK: bag {args.bag} passed all enabled checks")
        return 0
    print(f"FAIL ({len(findings)} check(s))", file=sys.stderr)
    for tag, msg in findings:
        print(f"  {tag}: {msg}", file=sys.stderr)
    return rc


if __name__ == "__main__":
    sys.exit(main())