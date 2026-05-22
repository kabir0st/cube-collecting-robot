"""
Generate audit diagrams for the robotics-labs-solutions project.

Outputs (PNG) into the same directory:
  - thread_topology.png        : threads + their shared structs
  - data_flow.png              : sensors -> fusion -> control -> motors
  - state_machine.png          : StateMachine transitions
  - lock_topology.png          : which mutex protects which struct
  - control_tick.png           : per-tick sequence inside loop_control

Run:
    python3 gen_diagrams.py
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
from matplotlib.lines import Line2D

HERE = os.path.dirname(os.path.abspath(__file__))


def save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, name), dpi=140, bbox_inches="tight")
    plt.close(fig)


# ---------- 1. Thread topology -----------------------------------------------
def thread_topology():
    fig, ax = plt.subplots(figsize=(13, 8))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 9)
    ax.axis("off")
    ax.set_title("Thread topology — who spawns whom, what they share",
                 fontsize=14, weight="bold")

    threads = {
        "main": (6.5, 8.0, "#cfe2ff", "main()\nspawns + joins"),
        "battery": (1.5, 6.0, "#d1e7dd", "loop_check_battery\n(2s tick)"),
        "localization": (4.5, 6.0, "#d1e7dd",
                         "loop_localization\n(spawns lidar + cox)"),
        "vision": (8.5, 6.0, "#d1e7dd",
                   "loop_vision_tracking\n(~camera FPS)"),
        "control": (11.5, 6.0, "#d1e7dd",
                    "loop_control\n(40ms tick)"),
        "lidar_frames": (3.0, 3.5, "#fff3cd",
                         "loop_collect_frames\n(lidar packet loop)"),
        "cox": (6.0, 3.5, "#fff3cd",
                "loop_scan_cox_estimation\n(~100ms tick)"),
    }
    for k, (x, y, c, t) in threads.items():
        ax.add_patch(FancyBboxPatch((x - 1.1, y - 0.55), 2.2, 1.1,
                                    boxstyle="round,pad=0.05",
                                    facecolor=c, edgecolor="black", lw=1.2))
        ax.text(x, y, t, ha="center", va="center", fontsize=9)

    # spawn arrows (main -> children)
    for child in ["battery", "localization", "vision", "control"]:
        x1, y1 = threads["main"][:2]
        x2, y2 = threads[child][:2]
        ax.add_patch(FancyArrowPatch((x1, y1 - 0.55), (x2, y2 + 0.55),
                                     arrowstyle="->", mutation_scale=14,
                                     color="black", lw=1.0))

    # localization spawns lidar + cox
    for child in ["lidar_frames", "cox"]:
        x1, y1 = threads["localization"][:2]
        x2, y2 = threads[child][:2]
        ax.add_patch(FancyArrowPatch((x1, y1 - 0.55), (x2, y2 + 0.55),
                                     arrowstyle="->", mutation_scale=14,
                                     color="darkblue", lw=1.0))

    # Shared structs row
    shared = [
        (1.6, 1.0, "INA219\n(battery thread only)"),
        (4.0, 1.0, "LidarData\n(lidar_frames -> cox, calib, telemetry)"),
        (7.6, 1.0, "KalmanFilter\n(cox.correct ↔ control.predict)"),
        (10.6, 1.0, "TrackerResult\n(vision -> control)"),
    ]
    for x, y, t in shared:
        ax.add_patch(FancyBboxPatch((x - 1.4, y - 0.45), 2.8, 0.9,
                                    boxstyle="round,pad=0.05",
                                    facecolor="#f8d7da", edgecolor="black", lw=1.0))
        ax.text(x, y, t, ha="center", va="center", fontsize=8.5)

    arrows_to_shared = [
        ("battery", (1.6, 1.45)),
        ("lidar_frames", (4.0, 1.45)),
        ("cox", (4.0, 1.45)),
        ("cox", (7.6, 1.45)),
        ("control", (7.6, 1.45)),
        ("vision", (10.6, 1.45)),
        ("control", (10.6, 1.45)),
    ]
    for src, dst in arrows_to_shared:
        x1, y1 = threads[src][:2]
        x2, y2 = dst
        ax.add_patch(FancyArrowPatch((x1, y1 - 0.55), (x2, y2),
                                     arrowstyle="->", mutation_scale=10,
                                     color="gray", lw=0.8, linestyle="--"))

    legend = [
        Line2D([0], [0], color="black", lw=1.0,
               label="main spawns child"),
        Line2D([0], [0], color="darkblue", lw=1.0,
               label="loop_localization spawns subthread"),
        Line2D([0], [0], color="gray", lw=0.8, linestyle="--",
               label="reads / writes shared struct"),
    ]
    ax.legend(handles=legend, loc="lower right", fontsize=8.5,
              frameon=True, framealpha=0.95)
    save(fig, "thread_topology.png")


# ---------- 2. Data flow ------------------------------------------------------
def data_flow():
    fig, ax = plt.subplots(figsize=(13, 7))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 7)
    ax.axis("off")
    ax.set_title("Per-tick data flow: sensors → fusion → mission → motors",
                 fontsize=14, weight="bold")

    nodes = {
        # sensors
        "LIDAR": (1.0, 5.5, "#fff3cd"),
        "Encoders": (1.0, 3.5, "#fff3cd"),
        "Camera": (1.0, 1.5, "#fff3cd"),
        "INA219": (1.0, 0.3, "#fff3cd"),

        # processing
        "loop_collect_frames\n(LidarData)": (3.5, 5.5, "#d1e7dd"),
        "find_walls\n(RANSAC)": (5.8, 6.0, "#d1e7dd"),
        "ref_map\n[LineSeg]": (8.0, 6.0, "#cfe2ff"),
        "cox.estimate\n(scan-match)": (5.8, 4.7, "#d1e7dd"),
        "Odometry.update\n(d, dθ, var)": (3.5, 3.5, "#d1e7dd"),
        "KF.predict": (5.8, 3.5, "#d1e7dd"),
        "KF.correct": (8.0, 4.7, "#d1e7dd"),
        "KF.snapshot\n(p, cov)": (8.0, 3.5, "#cfe2ff"),

        "loop_vision_tracking\n(HSV blob)": (3.5, 1.5, "#d1e7dd"),
        "TrackerResult\n(bearing, dist,\ncaptured, lost, grabbable)": (5.8, 1.5, "#cfe2ff"),

        "StateMachine.step": (10.0, 3.5, "#fde2e4"),
        "PurePursuit.compute": (10.0, 2.0, "#fde2e4"),
        "Motor.set_velocity": (12.0, 2.7, "#f8d7da"),
        "Battery monitor": (3.5, 0.3, "#d1e7dd"),
    }
    for k, (x, y, c) in nodes.items():
        w, h = 1.7, 0.7
        if "\n" in k and k.count("\n") >= 2:
            h = 0.95
        ax.add_patch(FancyBboxPatch((x - w/2, y - h/2), w, h,
                                    boxstyle="round,pad=0.05",
                                    facecolor=c, edgecolor="black", lw=1.0))
        ax.text(x, y, k, ha="center", va="center", fontsize=8.0)

    edges = [
        ("LIDAR", "loop_collect_frames\n(LidarData)"),
        ("loop_collect_frames\n(LidarData)", "find_walls\n(RANSAC)"),
        ("loop_collect_frames\n(LidarData)", "cox.estimate\n(scan-match)"),
        ("find_walls\n(RANSAC)", "ref_map\n[LineSeg]"),
        ("ref_map\n[LineSeg]", "cox.estimate\n(scan-match)"),
        ("cox.estimate\n(scan-match)", "KF.correct"),
        ("Encoders", "Odometry.update\n(d, dθ, var)"),
        ("Odometry.update\n(d, dθ, var)", "KF.predict"),
        ("KF.predict", "KF.snapshot\n(p, cov)"),
        ("KF.correct", "KF.snapshot\n(p, cov)"),
        ("Camera", "loop_vision_tracking\n(HSV blob)"),
        ("loop_vision_tracking\n(HSV blob)", "TrackerResult\n(bearing, dist,\ncaptured, lost, grabbable)"),
        ("KF.snapshot\n(p, cov)", "StateMachine.step"),
        ("TrackerResult\n(bearing, dist,\ncaptured, lost, grabbable)", "StateMachine.step"),
        ("StateMachine.step", "PurePursuit.compute"),
        ("StateMachine.step", "Motor.set_velocity"),
        ("PurePursuit.compute", "Motor.set_velocity"),
        ("INA219", "Battery monitor"),
    ]
    for a, b in edges:
        x1, y1, _ = nodes[a]
        x2, y2, _ = nodes[b]
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2),
                                     arrowstyle="->", mutation_scale=12,
                                     color="black", lw=0.9,
                                     connectionstyle="arc3,rad=0.05"))
    save(fig, "data_flow.png")


# ---------- 3. State machine --------------------------------------------------
def state_machine():
    fig, ax = plt.subplots(figsize=(13, 8))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 9)
    ax.axis("off")
    ax.set_title("StateMachine transitions (controller.cpp)",
                 fontsize=14, weight="bold")

    states = {
        "IDLE": (1.0, 7.5, "#e9ecef"),
        "SEARCH": (4.5, 7.5, "#d1e7dd"),
        "APPROACH": (8.0, 7.5, "#cff4fc"),
        "CARRY_HOME": (11.5, 7.5, "#fff3cd"),
        "RELEASE": (11.5, 4.5, "#fde2e4"),
        "TURN_AROUND": (8.0, 4.5, "#fde2e4"),
        "DONE": (4.5, 4.5, "#e9ecef"),
        "SAFE_STOP": (1.0, 4.5, "#f8d7da"),
        "GOTO": (6.25, 1.5, "#cfe2ff"),
    }
    for k, (x, y, c) in states.items():
        ax.add_patch(FancyBboxPatch((x - 1.0, y - 0.45), 2.0, 0.9,
                                    boxstyle="round,pad=0.05",
                                    facecolor=c, edgecolor="black", lw=1.2))
        ax.text(x, y, k, ha="center", va="center", fontsize=10,
                weight="bold")

    transitions = [
        ("IDLE", "SEARCH", "start()"),
        ("SEARCH", "APPROACH", "target visible &\nallow_engage"),
        ("APPROACH", "SEARCH", "timeout /\ngrip miss"),
        ("APPROACH", "CARRY_HOME", "captured\n(after grip)"),
        ("CARRY_HOME", "RELEASE", "at home"),
        ("CARRY_HOME", "SAFE_STOP", "plan fail /\ntimeout"),
        ("RELEASE", "TURN_AROUND", "released or\ntimeout"),
        ("TURN_AROUND", "SEARCH", "180° accum"),
        ("SEARCH", "SAFE_STOP", "attempts ≥ max /\ntimeout"),
        ("IDLE", "GOTO", "go_to()"),
        ("DONE", "GOTO", "go_to()"),
        ("SAFE_STOP", "GOTO", "go_to()"),
        ("GOTO", "DONE", "PP.has_arrived"),
        ("GOTO", "SAFE_STOP", "plan fail"),
    ]
    for src, dst, lbl in transitions:
        x1, y1, _ = states[src]
        x2, y2, _ = states[dst]
        rad = 0.10
        if (src, dst) in [("SEARCH", "APPROACH"), ("APPROACH", "SEARCH")]:
            rad = 0.18 if src == "SEARCH" else -0.18
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2),
                                     arrowstyle="->", mutation_scale=12,
                                     color="black", lw=0.9,
                                     connectionstyle=f"arc3,rad={rad}",
                                     shrinkA=18, shrinkB=18))
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2 + (0.25 if rad >= 0 else -0.25)
        ax.text(mx, my, lbl, ha="center", va="center", fontsize=7.5,
                style="italic",
                bbox=dict(boxstyle="round,pad=0.15",
                          facecolor="white", edgecolor="none", alpha=0.8))

    # Highlight problematic states (no timeout, etc.)
    for s in ["TURN_AROUND"]:
        x, y, _ = states[s]
        ax.add_patch(Rectangle((x - 1.05, y - 0.5), 2.1, 1.0,
                               fill=False, edgecolor="red", lw=2.0))
    ax.text(8.0, 3.7, "⚠ no hard timeout — motor jam wedges here",
            ha="center", color="red", fontsize=9, style="italic")
    save(fig, "state_machine.png")


# ---------- 4. Lock topology --------------------------------------------------
def lock_topology():
    fig, ax = plt.subplots(figsize=(12, 7))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 7)
    ax.axis("off")
    ax.set_title("Mutex / atomic protection map",
                 fontsize=14, weight="bold")

    items = [
        # (x, y, struct, fields, writers, readers, lock)
        (2.0, 5.5, "LidarData",
         "frames[3], buffer_id (atomic),\nsensor_noise (NOT atomic ⚠)",
         "loop_collect_frames",
         "cox, find_walls, telemetry",
         "lock (per-snapshot)"),
        (6.0, 5.5, "KalmanFilter",
         "_pose, _cov",
         "control (predict),\ncox (correct/reset)",
         "control, cox (snapshot)",
         "_m"),
        (10.0, 5.5, "TrackerResult",
         "has_target,\nbearing/dist/area (locked);\ncaptured/lost/grabbable (atomic ⚠ split)",
         "loop_vision_tracking",
         "loop_control (StateMachine.step)",
         "lock + atomics"),
        (2.0, 2.5, "StateMachine",
         "_state (atomic),\n_goto_*, _home_*,\n_approach_attempts",
         "loop_control (step),\nany thread (start/go_to/abort)",
         "loop_control + dashboard snap",
         "_m"),
        (6.0, 2.5, "PurePursuitController",
         "_path, _mode, _arrived",
         "StateMachine (set_path)",
         "loop_control (compute)",
         "_m"),
        (10.0, 2.5, "app::* atomics",
         "shutdown, lidar_ready,\nlidar_stop, obstacle_ahead (UNUSED ⚠),\nrelocalizing",
         "many",
         "many",
         "std::atomic<bool>"),
    ]
    colors = ["#fff3cd", "#d1e7dd", "#cfe2ff",
              "#fde2e4", "#f8d7da", "#e9ecef"]
    for (x, y, name, fields, w, r, lk), c in zip(items, colors):
        ax.add_patch(FancyBboxPatch((x - 1.9, y - 1.1), 3.8, 2.2,
                                    boxstyle="round,pad=0.05",
                                    facecolor=c, edgecolor="black", lw=1.2))
        ax.text(x, y + 0.85, name, ha="center", va="center",
                fontsize=10, weight="bold")
        ax.text(x, y + 0.30, f"fields: {fields}",
                ha="center", va="center", fontsize=7.5)
        ax.text(x, y - 0.25, f"W: {w}", ha="center", va="center", fontsize=7.5)
        ax.text(x, y - 0.60, f"R: {r}", ha="center", va="center", fontsize=7.5)
        ax.text(x, y - 0.95, f"lock: {lk}", ha="center", va="center",
                fontsize=7.5, style="italic")
    ax.text(6.0, 0.35,
            "⚠ = audit finding. See bugs-and-issues.md for details.",
            ha="center", color="red", fontsize=9, style="italic")
    save(fig, "lock_topology.png")


# ---------- 5. Per-tick control sequence -------------------------------------
def control_tick():
    fig, ax = plt.subplots(figsize=(11, 7))
    ax.set_xlim(0, 11)
    ax.set_ylim(0, 11)
    ax.axis("off")
    ax.set_title("loop_control 40 ms tick — sequence inside one iteration",
                 fontsize=14, weight="bold")

    steps = [
        (1, "sleep 40 ms (2 motor cycles)"),
        (2, "motor.get_encoders(l_mm, r_mm)"),
        (3, "motor.get_current_v_w(v_sm, w_sm)"),
        (4, "Odometry.update(l, r) -> OdomStep"),
        (5, "kf.predict(d, dθ, var_d, var_θ, cov_dθ)"),
        (6, "kf.snapshot(p, cov)"),
        (7, "StateMachine.step(p, dash)\n  reads TrackerResult.snapshot\n  emits Command (STOP / DIRECT / PATH_FOLLOW)"),
        (8, "if PATH_FOLLOW: pp.compute(p, v_cmd, w_cmd)"),
        (9, "if app::relocalizing: v_cmd = w_cmd = 0"),
        (10, "motor.set_velocity(v_cmd, w_cmd)"),
        (11, "publish dash + telemetry (10 Hz gated)"),
    ]
    y = 10.2
    for i, (n, t) in enumerate(steps):
        ax.add_patch(FancyBboxPatch((1.0, y - 0.45), 9.0, 0.85,
                                    boxstyle="round,pad=0.05",
                                    facecolor="#cfe2ff" if n in (7, 8) else "#e9ecef",
                                    edgecolor="black", lw=1.0))
        ax.text(1.4, y, f"{n}.", fontsize=10, weight="bold",
                ha="left", va="center")
        ax.text(2.0, y, t, fontsize=9, ha="left", va="center")
        if i < len(steps) - 1:
            ax.add_patch(FancyArrowPatch((5.5, y - 0.5),
                                         (5.5, y - 0.85),
                                         arrowstyle="->",
                                         mutation_scale=10,
                                         color="black", lw=0.8))
        y -= 0.95

    ax.text(5.5, 0.2,
            "Note: app::obstacle_ahead is set by loop_collect_frames but never read here.",
            ha="center", color="red", fontsize=8.5, style="italic")
    save(fig, "control_tick.png")


def main():
    thread_topology()
    data_flow()
    state_machine()
    lock_topology()
    control_tick()
    print("All diagrams generated.")


if __name__ == "__main__":
    main()
