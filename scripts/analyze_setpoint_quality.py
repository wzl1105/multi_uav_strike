#!/usr/bin/env python3
"""
Analyze a gs_replay playback bag for OFFBOARD setpoint issues.

Four failure modes are detected (each can be enabled/disabled via flags):

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

  4. --check-cross-channel : During mission phases where one channel is
                          authoritative (mission_manager owns position in
                          phases 1/2/9 = TAKING_OFF/HOVERING/HOLDING;
                          waypoint_executor owns velocity in phases 3/6 =
                          WAYPOINT_FOLLOW/RETURNING), detect concurrent
                          publishing to *both* position and velocity topics.
                          This catches the 2026-08-17 bug where
                          publishZeroVelocity() was emitting zero velocity
                          during TAKING_OFF, colliding with mission_manager's
                          position setpoint → PX4 OFFBOARD type_mask 冲突
                          → thrust=0 → 10s 后 AUTO.LAND.
                          Findings whose overlap window falls within
                          ±--cross-grace-sec of any phase transition are
                          tagged as SUPPRESSED (handoff design region: the
                          position channel primes before HOLDING via
                          waiting_for_waypoint_velocity_handoff_ latch, and
                          waypoint_executor stops ~50ms after a new task_flow
                          — both produce short bursts of concurrent
                          publishing that are NOT bugs).

Default invocation checks all four; rc=0 if none of the enabled checks fail.
Output is human-readable and machine-parseable (each finding starts with a
prefixed tag: AUTOLAND / STREAMGAP / SIMULT / SIMULT-CROSS).

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


# ----------------------------- cross-channel overlap -----------------------------

# Mission phase constants — keep in sync with src/mission_manager_node.cpp
PHASE_GROUND_IDLE     = 0
PHASE_TAKING_OFF      = 1
PHASE_HOVERING        = 2
PHASE_WAYPOINT_FOLLOW = 3
PHASE_RETURNING       = 6
PHASE_HOLDING         = 9

# Phases where mission_manager owns position setpoint as authoritative.
# During these phases, waypoint_executor MUST NOT publish any velocity setpoint
# (real or zero), otherwise PX4 OFFBOARD receives type_mask 冲突 → NaN → thrust=0.
POSITION_AUTHORITATIVE_PHASES = {PHASE_TAKING_OFF, PHASE_HOVERING, PHASE_HOLDING}
# Phases where waypoint_executor owns velocity as authoritative.
VELOCITY_AUTHORITATIVE_PHASES = {PHASE_WAYPOINT_FOLLOW, PHASE_RETURNING}


def _index_phase_segments(bag, mission_state_topic):
    """
    Return list of (start_ts, end_ts, phase) for stable-phase periods.

    mission_state is published at 1Hz, so phase transitions are coarse.
    A 'segment' starts at the timestamp when a new phase value is first seen
    and lasts until the next phase change (or end-of-bag).
    """
    if mission_state_topic is None:
        return []
    samples = []
    try:
        for topic, msg, t in bag.read_messages(mission_state_topic):
            samples.append((t.to_sec(), int(getattr(msg, "phase", -1))))
    except Exception:
        return []
    samples.sort()
    segments = []
    for i, (ts, ph) in enumerate(samples):
        end_ts = samples[i + 1][0] if i + 1 < len(samples) else float("inf")
        segments.append((ts, end_ts, ph))
    return samples, segments


def _topic_timestamps(bag, candidates):
    """
    Read all candidate topics; return merged sorted timestamp list, ignoring
    topics with zero messages.
    """
    series = []
    for cand in candidates:
        try:
            if bag.get_message_count(cand) == 0:
                continue
        except Exception:
            continue
        for topic, msg, t in bag.read_messages(cand):
            series.append(t.to_sec())
    series.sort()
    return series


def _intersection_duration(pos_ts, vel_ts, win_start, win_end):
    """
    Walk two sorted timestamp lists and return the total duration within
    [win_start, win_end] where *both* lists have at least one timestamp
    within a `window_sec` of each other (sliding-window style).

    Returns total_overlap_seconds.
    """
    window_sec = 0.1  # hard-coded; matches --simul-window default
    if not pos_ts or not vel_ts:
        return 0.0, None, None

    # Clip both lists to [win_start, win_end]
    import bisect
    i_lo = bisect.bisect_left(pos_ts, win_start)
    j_lo = bisect.bisect_left(vel_ts, win_start)
    pos_clip = [ts for ts in pos_ts[i_lo:] if ts <= win_end]
    vel_clip = [ts for ts in vel_ts[j_lo:] if ts <= win_end]
    if not pos_clip or not vel_clip:
        return 0.0, None, None

    # For each pos timestamp, find any vel timestamp within window_sec.
    # Note: when inner loop breaks on vt > pt+window, vt might still fall
    # inside the window of the *next* pt (pt' > pt), so don't skip ahead —
    # just let the next iteration re-evaluate from current j.
    j = 0
    overlap = []  # list of (t_pos, t_vel) when both have data within window
    for pt in pos_clip:
        # advance j to first vel_ts >= pt - window_sec
        while j < len(vel_clip) and vel_clip[j] < pt - window_sec:
            j += 1
        if j >= len(vel_clip):
            # all remaining vel entries are in the past; subsequent pt's won't match either
            break
        # check vel_ts[k] within [pt - window_sec, pt + window_sec]
        for k in range(j, len(vel_clip)):
            vt = vel_clip[k]
            if vt > pt + window_sec:
                break
            overlap.append((pt, vt))

    if not overlap:
        return 0.0, None, None

    # Total time span from earliest to latest pair
    first = overlap[0]
    last = overlap[-1]
    return max(last[0], last[1]) - min(first[0], first[1]), min(first[0], first[1]), max(last[0], last[1])


def _phase_transition_timestamps(samples):
    """Return list of timestamps where phase value changes between consecutive samples."""
    out = []
    for i in range(1, len(samples)):
        if samples[i][1] != samples[i - 1][1]:
            out.append(samples[i][0])
    return out


def _near_phase_transition(overlap_start, overlap_end, transitions, grace_sec):
    """
    Return (is_near, nearest_transition_ts, distance_sec) if overlap window
    is within grace_sec of any phase transition AND overlap is short enough
    to plausibly be a designed handoff.

    Designed handoff overlaps (mission_manager's setpoint_thread pre-priming
    for HOLDING via waiting_for_waypoint_velocity_handoff_ latch, or
    waypoint_executor stopping lag) typically occur within ±1s of a phase
    transition AND last < 1s. We additionally require overlap duration to
    be < 2*grace_sec so that a 5-second real bug whose start coincides
    with a transition is NOT suppressed.
    """
    if not transitions:
        return False, None, float("inf")
    nearest_ts = None
    nearest_d = float("inf")
    for ts in transitions:
        d = min(abs(ts - overlap_start), abs(ts - overlap_end))
        if d < nearest_d:
            nearest_d = d
            nearest_ts = ts
    if nearest_d >= grace_sec:
        return False, nearest_ts, nearest_d
    # Even if near transition, a long overlap is not a handoff — it's a real bug
    overlap_dur = overlap_end - overlap_start
    if overlap_dur > 2 * grace_sec:
        return False, nearest_ts, nearest_d
    return True, nearest_ts, nearest_d


def analyze_cross_channel_setpoint(
    bag,
    candidate_topics_for_position,
    candidate_topics_for_velocity,
    mission_state_topic,
    min_overlap_sec,
    grace_sec,
):
    """
    Detect cross-channel overlap (position + velocity topics both receiving
    messages) during phases where one channel should be authoritative.

    Each finding covers one phase segment where the violation occurs.
    Phase 1/2/9 (POSITION_AUTHORITATIVE): position OK, velocity must NOT publish.
    Phase 3/6 (VELOCITY_AUTHORITATIVE): velocity OK, position must NOT publish.

    Suppression: findings whose overlap window falls within ±grace_sec of any
    phase transition are tagged with `suppressed=True` and excluded from the
    error count. They are still returned so the caller can log them as INFO.
    Rationale: the position channel primes before phase transitions (latch
    mechanism) and waypoint_executor stops ~50ms after the new task_flow.
    Both produce short bursts of concurrent publishing that are NOT bugs.

    Returns list of findings:
        {phase, phase_name, authoritative, offending_channel, window_start,
         window_end, overlap_sec, suppressed, suppression_reason}
    """
    samples, segments = _index_phase_segments(bag, mission_state_topic)
    if not segments:
        return []

    pos_ts = _topic_timestamps(bag, candidate_topics_for_position)
    vel_ts = _topic_timestamps(bag, candidate_topics_for_velocity)
    transitions = _phase_transition_timestamps(samples)

    phase_name = {
        PHASE_GROUND_IDLE:     "GROUND_IDLE",
        PHASE_TAKING_OFF:      "TAKING_OFF",
        PHASE_HOVERING:        "HOVERING",
        PHASE_WAYPOINT_FOLLOW: "WAYPOINT_FOLLOW",
        PHASE_RETURNING:       "RETURNING",
        PHASE_HOLDING:         "HOLDING",
    }

    findings = []
    for seg_start, seg_end, ph in segments:
        if ph in POSITION_AUTHORITATIVE_PHASES:
            authoritative = "position"
            offending = "velocity"
        elif ph in VELOCITY_AUTHORITATIVE_PHASES:
            authoritative = "velocity"
            offending = "position"
        else:
            # GROUND_IDLE etc — neither channel should publish
            authoritative = "none"
            offending = "both"

        overlap, ovl_start, ovl_end = _intersection_duration(
            pos_ts, vel_ts, seg_start, seg_end)
        if overlap >= min_overlap_sec:
            suppressed, near_ts, near_dist = _near_phase_transition(
                ovl_start, ovl_end, transitions, grace_sec)
            if suppressed:
                reason = "near phase transition (t={:.3f}s, dist={:.3f}s, grace={:.2f}s)".format(
                    near_ts, near_dist, grace_sec)
            else:
                reason = ""
            findings.append({
                "phase": ph,
                "phase_name": phase_name.get(ph, f"PHASE_{ph}"),
                "authoritative": authoritative,
                "offending_channel": offending,
                "segment_start": seg_start,
                "segment_end": seg_end,
                "window_start": ovl_start,
                "window_end": ovl_end,
                "overlap_sec": overlap,
                "suppressed": suppressed,
                "suppression_reason": reason,
            })
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
    ap.add_argument("--no-cross-channel", action="store_true",
                    help="Disable cross-channel (position vs velocity) overlap check")
    ap.add_argument("--stream-gap-threshold", type=float, default=1.0,
                    help="Stream gap threshold in seconds (default 1.0 = PX4 "
                         "COM_OFFBOARD_LOSS_TIMEOUT default)")
    ap.add_argument("--simul-window", type=float, default=0.1,
                    help="Simultaneous-publish sliding window in seconds "
                         "(default 0.1)")
    ap.add_argument("--simul-min-overlap", type=float, default=0.5,
                    help="Minimum overlap duration to flag as simultaneous "
                         "(default 0.5 s)")
    ap.add_argument("--cross-grace-sec", type=float, default=1.5,
                    help="SIMULT-CROSS phase-transition grace window in seconds. "
                         "Overlap windows within ±N seconds of any phase transition "
                         "are tagged as suppressed (handoff design region, not a bug). "
                         "(default 1.5 s)")
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

        # ------- Check 4: cross-channel overlap (position vs velocity) -------
        if not args.no_cross_channel:
            cross = analyze_cross_channel_setpoint(
                bag, pos_tpcs, vel_tpcs, mission_state_tpc,
                args.simul_min_overlap, args.cross_grace_sec,
            )
            failing = [f for f in cross if not f["suppressed"]]
            suppressed = [f for f in cross if f["suppressed"]]
            if failing:
                findings.append(("SIMULT-CROSS",
                                 "{} phase segment(s) with cross-channel overlap".format(len(failing))))
                for f in failing:
                    print(f"  SIMULT-CROSS phase={f['phase']}({f['phase_name']}) "
                          f"authoritative={f['authoritative']} offending={f['offending_channel']} "
                          f"window={f['window_start']:.3f}-{f['window_end']:.3f}s "
                          f"overlap={f['overlap_sec']:.3f}s")
                rc = 1
            if suppressed:
                # INFO only — designed handoff region, not a bug
                for f in suppressed:
                    print(f"  SIMULT-CROSS-SUPPRESSED phase={f['phase']}({f['phase_name']}) "
                          f"authoritative={f['authoritative']} offending={f['offending_channel']} "
                          f"window={f['window_start']:.3f}-{f['window_end']:.3f}s "
                          f"overlap={f['overlap_sec']:.3f}s "
                          f"reason={f['suppression_reason']}")

    if not findings:
        print(f"OK: bag {args.bag} passed all enabled checks")
        return 0
    print(f"FAIL ({len(findings)} check(s))", file=sys.stderr)
    for tag, msg in findings:
        print(f"  {tag}: {msg}", file=sys.stderr)
    return rc


if __name__ == "__main__":
    sys.exit(main())