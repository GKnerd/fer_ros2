#!/usr/bin/env python3
"""Tracking report of a joint_trajectory_controller from a bag of its controller_state.

Splits the bag into motions (spans where the reference moves). Per motion it prints the
planned duration, how long the arm took to come to rest after the reference stopped,
the time MoveIt allows, and how far each joint lagged behind the reference.

Run inside the container (ROS sourced):
    python3 analyze_jtc_bag.py /home/fer_ros2/data/effort_traj_controller_rec --plot lag.png
"""
import argparse

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPIC = "/effort_trajectory_controller/controller_state"
MOVING_VEL = 1e-3  # rad/s, reference speed above which a motion is running
STOPPED_VEL = 0.01  # rad/s, JTC default stopped_velocity_tolerance
SCALING, MARGIN = 1.1, 0.5  # trajectory_execution in fer_moveit_launch.py
MIN_GAP = 0.05  # s, shorter pauses of the reference do not split a motion


def read_bag(uri, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(f"{topic} not in bag, topics: {sorted(types)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    msg_type = get_message(types[topic])

    names, rows = None, []
    while reader.has_next():
        _, data, t_wall = reader.read_next()
        m = deserialize_message(data, msg_type)
        names = names or list(m.joint_names)
        n = len(names)

        def vec(a):
            return np.asarray(a, float) if len(a) == n else np.full(n, np.nan)

        rows.append((
            m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, t_wall * 1e-9,
            vec(m.reference.positions), vec(m.reference.velocities),
            vec(m.feedback.positions), vec(m.feedback.velocities), vec(m.output.effort)))
    if not rows:
        raise SystemExit("no messages")
    t, t_wall, ref_p, ref_v, fb_p, fb_v, eff = (np.array(c) for c in zip(*rows))
    return names, dict(t=t, t_wall=t_wall, ref_p=ref_p, ref_v=ref_v, fb_p=fb_p,
                       fb_v=fb_v, eff=eff)


def find_motions(t, ref_v):
    moving = np.nanmax(np.abs(ref_v), axis=1) > MOVING_VEL
    edges = np.diff(moving.astype(int))
    starts = list(np.flatnonzero(edges == 1) + 1)
    ends = list(np.flatnonzero(edges == -1))
    if moving[0]:
        starts.insert(0, 0)
    if moving[-1]:
        ends.append(len(t) - 1)
    spans = []
    for s, e in zip(starts, ends):
        if spans and t[s] - t[spans[-1][1]] < MIN_GAP:
            spans[-1] = (spans[-1][0], e)
        else:
            spans.append((s, e))
    return spans


def report(names, d, spans):
    t, lag = d["t"], np.abs(d["ref_p"] - d["fb_p"])
    short = [n.replace("fer_", "") for n in names]
    print(f"{len(t)} messages, {t[-1] - t[0]:.2f} s controller time, "
          f"{d['t_wall'][-1] - d['t_wall'][0]:.2f} s wall time, {len(spans)} motion(s)\n")
    for i, (s, e) in enumerate(spans, 1):
        planned = t[e] - t[s]
        rest = np.flatnonzero(np.nanmax(np.abs(d["fb_v"][e:]), axis=1) < STOPPED_VEL)
        limit = planned * SCALING + MARGIN
        print(f"motion {i}: wall start {d['t_wall'][s]:.3f} (compare with launch.log)")
        print(f"  planned {planned:.2f} s, MoveIt allows {limit:.2f} s")
        if rest.size:
            settle = t[e + rest[0]] - t[e]
            verdict = "TOO SLOW" if planned + settle > limit else "ok"
            print(f"  at rest {settle:.2f} s after the reference stopped "
                  f"-> total {planned + settle:.2f} s  [{verdict}]")
        else:
            print("  never came to rest before the bag ended  [TOO SLOW]")
        print("  " + " ".join(f"{n:>8}" for n in ["joint"] + short))
        rows = [("max lag", lag[s:e + 1].max(axis=0)), ("end lag", lag[e]),
                ("max |eff|", np.nanmax(np.abs(d["eff"][s:e + 1]), axis=0))]
        for label, values in rows:
            print("  " + f"{label:>8} " + " ".join(f"{v:8.4f}" for v in values))
        print(f"  worst lag: {names[int(lag[s:e + 1].max(axis=0).argmax())]} "
              "(lag in rad, effort in Nm)\n")


def plot(names, d, spans, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = d["t"] - d["t"][0]
    fig, axes = plt.subplots(len(names), 1, sharex=True, figsize=(10, 2 * len(names)))
    for j, ax in enumerate(axes):
        ax.plot(t, d["ref_p"][:, j] - d["fb_p"][:, j], lw=0.8)
        for s, e in spans:
            ax.axvspan(t[s], t[e], color="0.85")
        ax.set_ylabel(f"{names[j]}\nlag [rad]", fontsize=8)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("controller time since bag start [s] (grey: reference moving)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"plot written to {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", help="bag directory")
    parser.add_argument("--topic", default=TOPIC)
    parser.add_argument("--plot", metavar="PNG", help="write the per-joint lag over time")
    args = parser.parse_args()

    names, d = read_bag(args.bag, args.topic)
    spans = find_motions(d["t"], d["ref_v"])
    report(names, d, spans)
    if args.plot:
        plot(names, d, spans, args.plot)


if __name__ == "__main__":
    main()
