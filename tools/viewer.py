#!/usr/bin/env python3
"""Live telemetry viewer for the robot.

Listens on UDP for newline-delimited JSON packets from src/telemetry.cpp and
draws them in a matplotlib window: reference map (walls), live LIDAR scan,
spline path, active target, goal, and KF pose with 1-sigma ellipse + trail.

Run: python3 tools/viewer.py
"""

import json
import math
import socket
import threading
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.patches import Ellipse

PORT = 5005
BIND = "0.0.0.0"
TRAIL_LEN = 600
SCAN_MAX_R_MM = 6000.0


class TelemetryState:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_walls = None        # (N,4) ax,ay,bx,by
        self.path = None             # (M,3) x,y,theta
        self.pose = None             # (x,y,theta)
        self.cov = None              # (sx,sy,sth)  1-sigma
        self.goal = None             # (x,y,theta)
        self.target = None           # (x,y,theta)
        self.scan_world = None       # (K,2) x,y in world
        self.trail = deque(maxlen=TRAIL_LEN)
        self.have_any = False
        self.have_map = False

    def update(self, packet):
        t = packet.get("type")
        with self.lock:
            if t == "map":
                walls = packet.get("walls") or []
                if walls:
                    self.map_walls = np.asarray(walls, dtype=float)
                    self.have_map = True
            elif t == "path":
                pts = packet.get("points") or []
                self.path = np.asarray(pts, dtype=float) if pts else None
            elif t == "state":
                pose = packet.get("pose")
                if pose:
                    self.pose = tuple(pose)
                    self.trail.append((pose[0], pose[1]))
                cov = packet.get("cov")
                if cov:
                    self.cov = tuple(cov)
                goal = packet.get("goal")
                if goal:
                    self.goal = tuple(goal)
                tgt = packet.get("target")
                if tgt:
                    self.target = tuple(tgt)
                # Transform scan from robot frame -> world frame using pose.
                scan = packet.get("scan") or []
                if scan and self.pose is not None:
                    a = np.asarray([s[0] for s in scan], dtype=float)
                    r = np.asarray([s[1] for s in scan], dtype=float)
                    mask = (np.isfinite(a) & np.isfinite(r)
                            & (r > 0) & (r < SCAN_MAX_R_MM))
                    a = a[mask]
                    r = r[mask]
                    px, py, pth = self.pose
                    # LIDAR ang is in the robot frame (0 = forward, +ccw).
                    wx = px + r * np.cos(pth + a)
                    wy = py + r * np.sin(pth + a)
                    self.scan_world = np.column_stack([wx, wy])
                else:
                    self.scan_world = None
            self.have_any = True


def udp_thread(state: TelemetryState):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((BIND, PORT))
    sock.settimeout(1.0)
    print(f"[viewer] listening on udp://{BIND}:{PORT}")
    buf = b""
    while True:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        # One datagram = one JSON object terminated with \n.
        for line in (buf + data).splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                pkt = json.loads(line)
            except json.JSONDecodeError:
                continue
            state.update(pkt)
        buf = b""


def main():
    state = TelemetryState()
    t = threading.Thread(target=udp_thread, args=(state,), daemon=True)
    t.start()

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.grid(True, alpha=0.3)
    ax.set_title("Robot telemetry — waiting for data…")
    # Fixed 3 m x 3 m window centered on origin (mm).
    ax.set_xlim(-1500, 1500)
    ax.set_ylim(-1500, 1500)

    walls_artist = LineCollection([], colors="gray", linewidths=2.0, zorder=1)
    ax.add_collection(walls_artist)

    (scan_artist,) = ax.plot([], [], ".", color="crimson", markersize=2.5,
                             alpha=0.7, zorder=2, label="LIDAR")
    (trail_artist,) = ax.plot([], [], "-", color="steelblue", alpha=0.35,
                              linewidth=1.0, zorder=3)
    (path_artist,) = ax.plot([], [], "--", color="royalblue", linewidth=1.5,
                             zorder=4, label="spline")
    (target_artist,) = ax.plot([], [], "o", color="royalblue", markersize=8,
                               markerfacecolor="none", markeredgewidth=1.6,
                               zorder=6, label="lookahead")
    (goal_artist,) = ax.plot([], [], "x", color="forestgreen", markersize=12,
                             markeredgewidth=2.2, zorder=6, label="goal")
    (robot_artist,) = ax.plot([], [], "o", color="black", markersize=7,
                              zorder=7, label="robot")
    heading_artist = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                                 arrowprops=dict(arrowstyle="->",
                                                 color="black", lw=1.8),
                                 zorder=7)
    ellipse_artist = Ellipse((0, 0), 0, 0, angle=0, fill=False,
                             edgecolor="black", lw=1.0, alpha=0.6, zorder=5)
    ax.add_patch(ellipse_artist)

    ax.legend(loc="upper right", fontsize=8)

    def update(_frame):
        with state.lock:
            walls = state.map_walls
            scan = state.scan_world
            path = state.path
            pose = state.pose
            cov = state.cov
            goal = state.goal
            tgt = state.target
            trail = list(state.trail)
            have_any = state.have_any
            have_map = state.have_map

        if have_map and walls is not None:
            segs = walls.reshape(-1, 2, 2)  # (N,2,2): [[ax,ay],[bx,by]]
            walls_artist.set_segments(segs)

        if scan is not None and len(scan):
            scan_artist.set_data(scan[:, 0], scan[:, 1])
        else:
            scan_artist.set_data([], [])

        if path is not None and len(path):
            path_artist.set_data(path[:, 0], path[:, 1])
        else:
            path_artist.set_data([], [])

        if trail:
            tr = np.asarray(trail)
            trail_artist.set_data(tr[:, 0], tr[:, 1])
        else:
            trail_artist.set_data([], [])

        if pose is not None:
            x, y, th = pose
            robot_artist.set_data([x], [y])
            L = 150.0
            heading_artist.xy = (x + L * math.cos(th), y + L * math.sin(th))
            heading_artist.set_position((x, y))
        else:
            robot_artist.set_data([], [])

        if pose is not None and cov is not None:
            sx, sy, _ = cov
            # 2-sigma rectangle-aligned ellipse (full cross-covariance is not
            # sent; this is a quick visual cue, not a metrology tool).
            ellipse_artist.center = (pose[0], pose[1])
            ellipse_artist.width = max(20.0, 4.0 * sx)
            ellipse_artist.height = max(20.0, 4.0 * sy)
            ellipse_artist.angle = 0.0

        if tgt is not None:
            target_artist.set_data([tgt[0]], [tgt[1]])
        else:
            target_artist.set_data([], [])

        if goal is not None:
            goal_artist.set_data([goal[0]], [goal[1]])
        else:
            goal_artist.set_data([], [])

        if have_any:
            ax.set_title(f"Robot telemetry — pose: "
                         f"({pose[0]:.0f}, {pose[1]:.0f}, "
                         f"{math.degrees(pose[2]):.1f}°)"
                         if pose else "Robot telemetry")

        return (walls_artist, scan_artist, path_artist, target_artist,
                goal_artist, robot_artist, trail_artist, ellipse_artist)

    anim = FuncAnimation(fig, update, interval=50, blit=False,
                         cache_frame_data=False)
    fig._anim = anim  # keep a reference so GC doesn't drop the animation
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
