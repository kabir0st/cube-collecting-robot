"""
Generate audit diagrams for the robotics-labs-solutions project.

Outputs (PNG) into the same directory:
  - thread_topology.png        : threads + their shared structs
  - data_flow.png              : sensors -> fusion -> control -> motors
  - state_machine.png          : top-level StateMachine transitions
  - search_substates.png       : SimpleSearch LOOK_LEFT / LOOK_RIGHT / FORWARD
  - approach_substates.png     : chase -> push -> stop tick -> grip -> slew
  - carry_substates.png        : NORMAL <-> REVERSING -> CURLING
  - scenarios.png              : multi-scenario sequence overview
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

    for child in ["battery", "localization", "vision", "control"]:
        x1, y1 = threads["main"][:2]
        x2, y2 = threads[child][:2]
        ax.add_patch(FancyArrowPatch((x1, y1 - 0.55), (x2, y2 + 0.55),
                                     arrowstyle="->", mutation_scale=14,
                                     color="black", lw=1.0))

    for child in ["lidar_frames", "cox"]:
        x1, y1 = threads["localization"][:2]
        x2, y2 = threads[child][:2]
        ax.add_patch(FancyArrowPatch((x1, y1 - 0.55), (x2, y2 + 0.55),
                                     arrowstyle="->", mutation_scale=14,
                                     color="darkblue", lw=1.0))

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
        "LIDAR": (1.0, 5.5, "#fff3cd"),
        "Encoders": (1.0, 3.5, "#fff3cd"),
        "Camera": (1.0, 1.5, "#fff3cd"),
        "INA219": (1.0, 0.3, "#fff3cd"),

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


# ---------- 3. Top-level state machine ---------------------------------------
def state_machine():
    """Two-panel layout: mission loop on top, lab/control + terminals on bottom.

    Splitting them avoids the arrow cluster between IDLE/DONE/SAFE_STOP
    and the lab states.
    """
    fig, ax = plt.subplots(figsize=(18, 12))
    ax.set_xlim(0, 18)
    ax.set_ylim(0, 13)
    ax.axis("off")
    ax.set_title(
        "StateMachine — top-level transitions (controller.cpp)",
        fontsize=15, weight="bold")

    # Background panels
    ax.add_patch(Rectangle((0.4, 6.6), 17.2, 5.6, facecolor="#f8f9fa",
                           edgecolor="#aab", lw=1.0, linestyle="--"))
    ax.text(0.7, 11.95, "Mission loop", fontsize=10.5, weight="bold",
            color="#555")
    ax.add_patch(Rectangle((0.4, 0.7), 17.2, 5.5, facecolor="#f8f9fa",
                           edgecolor="#aab", lw=1.0, linestyle="--"))
    ax.text(0.7, 5.95, "Lab / control / terminals", fontsize=10.5,
            weight="bold", color="#555")

    states = {
        # Mission loop (top): happy path arc + recovery row underneath
        "SEARCH":       (3.5, 10.6, "#d1e7dd"),
        "APPROACH":     (8.5, 10.6, "#cff4fc"),
        "CARRY_HOME":   (13.5, 10.6, "#fff3cd"),
        "TURN_AROUND":  (8.5, 7.8,  "#fde2e4"),
        "RELEASE":      (13.5, 7.8, "#fde2e4"),
        # Lab / control panel (bottom)
        "IDLE":      (3.0, 4.5, "#e9ecef"),
        "DONE":      (3.0, 2.0, "#e9ecef"),
        "SAFE_STOP": (15.0, 2.0, "#f8d7da"),
        "GOTO":      (9.0, 4.5, "#cfe2ff"),
        "TEST":      (15.0, 4.5, "#cfe2ff"),
    }
    for k, (x, y, c) in states.items():
        ax.add_patch(FancyBboxPatch((x - 1.3, y - 0.55), 2.6, 1.1,
                                    boxstyle="round,pad=0.06",
                                    facecolor=c, edgecolor="black", lw=1.3))
        ax.text(x, y, k, ha="center", va="center", fontsize=11,
                weight="bold")

    def arrow(src, dst, lbl, rad=0.0, lbl_off=(0.0, 0.0), fontsize=7.5):
        x1, y1, _ = states[src]
        x2, y2, _ = states[dst]
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2),
                                     arrowstyle="->", mutation_scale=13,
                                     color="black", lw=1.0,
                                     connectionstyle=f"arc3,rad={rad}",
                                     shrinkA=24, shrinkB=24))
        mx = (x1 + x2) / 2 + lbl_off[0]
        my = (y1 + y2) / 2 + lbl_off[1]
        # Offset by curve perpendicular so the label sits on the arc.
        perp_x = -(y2 - y1) * rad * 0.5
        perp_y = (x2 - x1) * rad * 0.5
        ax.text(mx + perp_x, my + perp_y, lbl, ha="center", va="center",
                fontsize=fontsize, style="italic",
                bbox=dict(boxstyle="round,pad=0.18",
                          facecolor="white", edgecolor="#ccc", lw=0.4,
                          alpha=0.95))

    # Mission loop
    arrow("SEARCH", "APPROACH",
          "target visible &\nnot filtered",
          rad=-0.15, lbl_off=(0, 0.55))
    arrow("APPROACH", "SEARCH",
          "vision lost > 3 s /\napproach timeout (budget+1) /\n"
          "dead-reckon arrival",
          rad=-0.30, lbl_off=(0, -0.80))
    arrow("APPROACH", "CARRY_HOME",
          "grip done (servo slew)\nOR snap.captured (pre-grip)",
          rad=-0.15, lbl_off=(0, 0.55))
    arrow("CARRY_HOME", "RELEASE",
          "PP arrived & inside _home_margin\n"
          "OR stuck × max (release in place)",
          rad=0.0, lbl_off=(0.6, 0.0))
    arrow("RELEASE", "TURN_AROUND",
          "back-up done /\nhard timeout (5 s)",
          rad=0.0, lbl_off=(0, 0.30))
    arrow("TURN_AROUND", "SEARCH",
          "|θ−home_θ| < 3° /\ntimeout (8 s)\n[via _enter_search]",
          rad=-0.15, lbl_off=(-0.2, 0.0))
    arrow("CARRY_HOME", "SAFE_STOP",
          "carry timeout\n(120 s)",
          rad=0.25, lbl_off=(-1.4, -2.4))
    # SEARCH -> CARRY_HOME on snap.captured is a rare re-entry edge;
    # documented in state-machine.md / scenarios.md and intentionally
    # omitted from the diagram to keep it readable.

    # Lab / control panel
    arrow("IDLE", "SEARCH",
          "start()\n[via _enter_search]",
          rad=-0.20, lbl_off=(-0.3, 1.2))
    arrow("IDLE", "GOTO", "go_to()", rad=0.0, lbl_off=(0, 0.25))
    arrow("IDLE", "TEST", "start_test()",
          rad=-0.20, lbl_off=(0, 0.40))
    arrow("DONE", "GOTO", "go_to()", rad=-0.20, lbl_off=(-0.3, 0.6))
    arrow("DONE", "TEST", "start_test()",
          rad=-0.20, lbl_off=(0, 0.30))
    arrow("DONE", "SEARCH", "start()",
          rad=-0.30, lbl_off=(-1.2, 1.0))
    arrow("SAFE_STOP", "GOTO", "go_to()",
          rad=0.20, lbl_off=(0, 0.40))
    arrow("SAFE_STOP", "TEST", "start_test()",
          rad=-0.20, lbl_off=(0, 0.30))
    arrow("GOTO", "DONE",
          "halt-done &\n_post_goto = DONE",
          rad=-0.20, lbl_off=(-0.4, 0.25))
    arrow("GOTO", "SEARCH",
          "halt-done &\n_post_goto = SEARCH\n(_enter_search chain)",
          rad=0.30, lbl_off=(-0.4, 1.9))
    arrow("GOTO", "SAFE_STOP", "plan fail",
          rad=-0.10, lbl_off=(0, 0.25))
    arrow("TEST", "DONE", "all legs done",
          rad=0.30, lbl_off=(0.4, 0.35))
    arrow("TEST", "SAFE_STOP", "plan fail",
          rad=0.0, lbl_off=(0.4, 0.0))

    ax.text(9.0, 0.30,
            "abort() (any state) → SAFE_STOP   "
            "│   halt-and-localize: GOTO/TEST hold v=0 for 300–700 ms "
            "until σ_xy ≤ 30 mm before declaring arrival",
            ha="center", fontsize=8.8, style="italic", color="#444")
    save(fig, "state_machine.png")


# ---------- 4. SEARCH sub-states ---------------------------------------------
def search_substates():
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis("off")
    ax.set_title("SEARCH sub-state machine (_ssearch in controller.cpp)",
                 fontsize=14, weight="bold")

    P = {
        "INIT":     (1.8, 7.3, "#e9ecef",
                     "INIT\nlatch ref =\npose.theta"),
        "LOOK_L":   (5.5, 7.3, "#d1e7dd",
                     "LOOK_LEFT\nω = +0.4 rad/s\nuntil θ = ref+π/2"),
        "LOOK_R":   (9.5, 7.3, "#d1e7dd",
                     "LOOK_RIGHT\nω = −0.4 rad/s\naccum −π"),
        "FWD":      (13.5, 7.3, "#d1e7dd",
                     "FORWARD\nrealign to ref →\ndrive 800 mm"),
        "REALIGN":  (13.5, 4.0, "#fff3cd",
                     "REALIGN\nhysteresis-guarded\n9° enter / 3° exit\n(stable 3 ticks)"),
        "DODGE":    (9.5, 4.0, "#fde2e4",
                     "OBSTACLE DODGE\nrotate toward\nfreer half-arc\n(cap at π)"),
        "TARGET":   (5.5, 4.0, "#cff4fc",
                     "TARGET DETECTED\n→ APPROACH\n(unless filtered)"),
        "XFAR":     (13.5, 1.0, "#f8d7da",
                     "X-FAR TURN-BACK\npose.x ≥ 2000 &\ncos(ref) > 0\n(flip ref by π)"),
    }
    for k, (x, y, c, label) in P.items():
        ax.add_patch(FancyBboxPatch((x - 1.75, y - 0.85), 3.5, 1.7,
                                    boxstyle="round,pad=0.06",
                                    facecolor=c, edgecolor="black", lw=1.1))
        ax.text(x, y, label, ha="center", va="center", fontsize=8.6)

    def edge(src, dst, lbl, rad=0.0, lbl_off=(0, 0)):
        x1, y1, _, _ = P[src]
        x2, y2, _, _ = P[dst]
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2),
                                     arrowstyle="->", mutation_scale=11,
                                     color="black", lw=0.9,
                                     connectionstyle=f"arc3,rad={rad}",
                                     shrinkA=34, shrinkB=34))
        if lbl:
            mx = (x1 + x2) / 2 + lbl_off[0]
            my = (y1 + y2) / 2 + lbl_off[1]
            ax.text(mx, my, lbl, ha="center", va="center", fontsize=7.5,
                    style="italic",
                    bbox=dict(boxstyle="round,pad=0.16",
                              facecolor="white", edgecolor="#ccc",
                              lw=0.4, alpha=0.95))

    # Top-row sweep cycle
    edge("INIT", "LOOK_L", "", rad=0.0)
    edge("LOOK_L", "LOOK_R", "err ≤ 0", rad=0.0, lbl_off=(0, 0.35))
    edge("LOOK_R", "FWD", "rot ≤ −π", rad=0.0, lbl_off=(0, 0.35))
    edge("FWD", "LOOK_L", "leg complete\n(800 mm)",
         rad=-0.45, lbl_off=(0, 1.6))

    # FORWARD ↔ REALIGN (column on the right)
    edge("FWD", "REALIGN", "|err| > 9°",
         rad=0.10, lbl_off=(0.7, 0))
    edge("REALIGN", "FWD", "|err| < 3°\nfor 3 ticks",
         rad=0.10, lbl_off=(-0.8, 0))

    # FORWARD ↔ DODGE
    edge("FWD", "DODGE", "app::obstacle_ahead",
         rad=0.20, lbl_off=(0.5, 0.7))
    edge("DODGE", "REALIGN", "obstacle clears",
         rad=0.0, lbl_off=(0, 0.30))
    edge("DODGE", "LOOK_L", "rot ≥ π\n(give up,\nrestart sweep)",
         rad=-0.30, lbl_off=(-1.5, 1.7))

    # X-far turn-back
    edge("FWD", "XFAR", "pose.x ≥ 2000\n& cos(ref) > 0",
         rad=0.0, lbl_off=(0.6, 0))
    edge("XFAR", "LOOK_R", "flip ref by π,\njump to LOOK_R",
         rad=-0.45, lbl_off=(-2.4, 0.5))

    # Target detection — fires from any sweep phase. Shown from LOOK_L
    # as a representative entry.
    edge("LOOK_L", "TARGET", "any tick:\nsnap.has_target & !lost",
         rad=0.0, lbl_off=(0.8, 0))

    ax.text(8.0, 0.10,
            "Skip-filters before APPROACH: at-home / behind-home / "
            "obstacle-block / failed-target (within 200 mm & 30 s). "
            "Target check fires from every sweep phase, not just LOOK_LEFT.",
            ha="center", fontsize=8.7, style="italic", color="#444")
    save(fig, "search_substates.png")


# ---------- 5. APPROACH sub-phases -------------------------------------------
def approach_substates():
    """Linear layout: chase → push → stop → grip → wait, with dropouts to
    DEAD-RECKON and exits to SEARCH/CARRY_HOME stacked on a separate row.
    """
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.axis("off")
    ax.set_title("APPROACH sub-phases (controller.cpp:861-1073)",
                 fontsize=14, weight="bold")

    P = {
        "CHASE":      (2.2, 7.0, "#cff4fc",
                       "CHASE\nvisual servo:\nv = 30, ω = k·bearing"),
        "PUSH":       (6.0, 7.0, "#d1e7dd",
                       "PUSH\ngrabbable rising;\nv = 20 mm/s\nfor 30 mm / 2 s"),
        "STOP_TICK":  (9.5, 7.0, "#fff3cd",
                       "STOP TICK\nv = 0;\none tick to\ndecelerate"),
        "GRIP_FIRE":  (13.0, 7.0, "#fff3cd",
                       "GRIP FIRE\nservo_grab():\nlatch target\nangle"),
        "GRIP_WAIT":  (13.0, 3.5, "#fde2e4",
                       "GRIP WAIT\nv = 0 until\nservo_done(0)\n/ 3 s watchdog"),
        "DEAD_RECKON":(2.2, 3.5, "#fde2e4",
                       "DEAD-RECKON\nvision dropped,\ndrive toward\nlatched target"),
        "TO_CARRY":   (9.5, 1.2, "#cfe2ff",
                       "→ CARRY_HOME"),
        "TO_SEARCH":  (2.2, 1.2, "#d1e7dd",
                       "→ SEARCH\n(no budget if\nperception gap)"),
    }
    for k, (x, y, c, label) in P.items():
        ax.add_patch(FancyBboxPatch((x - 1.6, y - 0.85), 3.2, 1.7,
                                    boxstyle="round,pad=0.06",
                                    facecolor=c, edgecolor="black", lw=1.1))
        ax.text(x, y, label, ha="center", va="center", fontsize=8.6)

    def edge(src, dst, lbl, rad=0.0, lbl_off=(0, 0), fontsize=7.5):
        x1, y1, _, _ = P[src]
        x2, y2, _, _ = P[dst]
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2),
                                     arrowstyle="->", mutation_scale=11,
                                     color="black", lw=0.9,
                                     connectionstyle=f"arc3,rad={rad}",
                                     shrinkA=34, shrinkB=34))
        mx = (x1 + x2) / 2 + lbl_off[0]
        my = (y1 + y2) / 2 + lbl_off[1]
        ax.text(mx, my, lbl, ha="center", va="center", fontsize=fontsize,
                style="italic",
                bbox=dict(boxstyle="round,pad=0.16",
                          facecolor="white", edgecolor="#ccc", lw=0.4,
                          alpha=0.95))

    # Happy linear path
    edge("CHASE", "PUSH", "snap.grabbable", rad=0.0, lbl_off=(0, 0.30))
    edge("PUSH", "STOP_TICK",
         "pushed ≥ 30 mm\nor 2 s elapsed",
         rad=0.0, lbl_off=(0, 0.45))
    edge("STOP_TICK", "GRIP_FIRE", "next tick",
         rad=0.0, lbl_off=(0, 0.30))
    edge("GRIP_FIRE", "GRIP_WAIT", "_grip_fired",
         rad=0.0, lbl_off=(0.3, 0))
    edge("GRIP_WAIT", "TO_CARRY", "slew done /\nwatchdog",
         rad=-0.20, lbl_off=(1.0, 0.6))

    # Dropouts / exits
    edge("CHASE", "DEAD_RECKON",
         "vision lost &\nlatched target",
         rad=-0.20, lbl_off=(-0.9, 0))
    edge("DEAD_RECKON", "CHASE", "vision back",
         rad=-0.20, lbl_off=(0.9, 0))
    edge("DEAD_RECKON", "TO_SEARCH",
         "reached arrival\nwithout re-acquire",
         rad=0.0, lbl_off=(0.3, 0))
    edge("CHASE", "TO_SEARCH",
         "vision lost > 3 s\n(direct, no staging)\n"
         "OR t_in > 20 s\n(budget +1)",
         rad=0.35, lbl_off=(-2.7, -2.8))
    edge("CHASE", "STOP_TICK",
         "rare: snap.captured\n(pre-grip)",
         rad=-0.40, lbl_off=(0, 1.4))

    ax.text(8.0, 0.10,
            "Reactive omega adds ±gain/dist bias per snap.obstacles "
            "entry in front of target. "
            "Obstacle gate is suppressed; visual servo handles avoidance.",
            ha="center", fontsize=8.7, style="italic", color="#444")
    save(fig, "approach_substates.png")


# ---------- 6. CARRY_HOME sub-states -----------------------------------------
def carry_substates():
    fig, ax = plt.subplots(figsize=(15, 9))
    ax.set_xlim(0, 15)
    ax.set_ylim(0, 9)
    ax.axis("off")
    ax.set_title("CARRY_HOME stuck-recovery sub-state machine",
                 fontsize=14, weight="bold")

    P = {
        "NORMAL":    (3.0, 7.0, "#d1e7dd",
                      "NORMAL\nfollow spline\nplanted on entry"),
        "STUCK":     (7.5, 7.0, "#fff3cd",
                      "STUCK?\nmoved < 30 mm\nin 3 s"),
        "REVERSING": (12.0, 7.0, "#fde2e4",
                      "REVERSING\nv = −50 mm/s\nfor 200 mm / 6 s"),
        "CURLING":   (12.0, 3.5, "#fde2e4",
                      "CURLING\nspline w/ midpoint\noffset ±250 mm\n(L then R)"),
        "ARRIVED":   (3.0, 3.5, "#cfe2ff",
                      "PP ARRIVED &\ninside _home_margin\n(spline done)"),
        "RIP":       (7.5, 3.5, "#f8d7da",
                      "RELEASE IN PLACE\nstuck × max attempts\n(drop wherever)"),
        "TO_RELEASE":(5.0, 1.0, "#fde2e4",
                      "→ RELEASE"),
        "TO_SAFE":   (10.0, 1.0, "#f8d7da",
                      "→ SAFE_STOP\n(carry timeout 120 s)"),
    }
    for k, (x, y, c, label) in P.items():
        ax.add_patch(FancyBboxPatch((x - 1.8, y - 0.85), 3.6, 1.7,
                                    boxstyle="round,pad=0.06",
                                    facecolor=c, edgecolor="black", lw=1.1))
        ax.text(x, y, label, ha="center", va="center", fontsize=8.6)

    def edge(src, dst, lbl, rad=0.0, lbl_off=(0, 0)):
        x1, y1, _, _ = P[src]
        x2, y2, _, _ = P[dst]
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2),
                                     arrowstyle="->", mutation_scale=11,
                                     color="black", lw=0.9,
                                     connectionstyle=f"arc3,rad={rad}",
                                     shrinkA=34, shrinkB=34))
        if lbl:
            mx = (x1 + x2) / 2 + lbl_off[0]
            my = (y1 + y2) / 2 + lbl_off[1]
            ax.text(mx, my, lbl, ha="center", va="center", fontsize=7.5,
                    style="italic",
                    bbox=dict(boxstyle="round,pad=0.16",
                              facecolor="white", edgecolor="#ccc",
                              lw=0.4, alpha=0.95))

    edge("NORMAL", "STUCK", "", rad=-0.10)
    edge("STUCK", "NORMAL", "moving", rad=-0.30, lbl_off=(0, 0.55))
    edge("STUCK", "REVERSING", "stuck &\nattempts < 2",
         rad=0.0, lbl_off=(0, 0.40))
    edge("REVERSING", "CURLING", "reverse done\n(curl L then R)",
         rad=0.0, lbl_off=(0.6, 0))
    edge("CURLING", "STUCK", "drive new\nspline → re-watch",
         rad=-0.30, lbl_off=(-1.5, -1.5))
    edge("STUCK", "RIP", "stuck &\nattempts = 2",
         rad=0.0, lbl_off=(0.3, 0))
    edge("RIP", "TO_RELEASE", "drop box wherever",
         rad=-0.15, lbl_off=(-0.2, 0))
    edge("NORMAL", "ARRIVED", "spline complete\n(home_margin)",
         rad=0.0, lbl_off=(0.4, 0))
    edge("CURLING", "ARRIVED", "spline complete",
         rad=0.30, lbl_off=(0, -1.6))
    edge("ARRIVED", "TO_RELEASE", "",
         rad=-0.15)
    edge("STUCK", "TO_SAFE", "120 s elapsed\n(global timeout)",
         rad=0.35, lbl_off=(2.3, -2.5))

    ax.text(7.5, 0.12,
            "Lidar obstacle gate is NOT suppressed in CARRY_HOME. "
            "A wall ahead zeros v; the stuck-watcher trips after 3 s; "
            "recovery reverses then curls around.",
            ha="center", fontsize=8.7, style="italic", color="#444")
    save(fig, "carry_substates.png")


# ---------- 7. Scenarios timeline --------------------------------------------
SCENARIOS = [
    # (title, color, [(label, fraction_of_width)])
    ("S1. Happy path", "#d1e7dd", [
        ("IDLE", 0.04), ("_enter_search → GOTO (stage)", 0.10),
        ("SEARCH (sweep + detect)", 0.18),
        ("APPROACH (chase → push → grip)", 0.16),
        ("CARRY_HOME (spline → home)", 0.18),
        ("RELEASE (back up 80 mm)", 0.10),
        ("TURN_AROUND (realign to home_θ)", 0.08),
        ("_enter_search → SEARCH (next box)", 0.16),
    ]),
    ("S2. Vision flicker mid-approach", "#cff4fc", [
        ("SEARCH", 0.20),
        ("APPROACH/CHASE", 0.18),
        ("vision lost (latched target dead-reckon, <3 s)", 0.18),
        ("CHASE re-acquires", 0.12),
        ("PUSH + GRIP", 0.12),
        ("CARRY_HOME", 0.20),
    ]),
    ("S3. Box wedged — failed attempt", "#fff3cd", [
        ("SEARCH (engage)", 0.16),
        ("APPROACH (20 s timeout, attempts+1)", 0.30),
        ("memorize failed-target world XY", 0.06),
        ("SEARCH (skips same XY for 30 s)", 0.24),
        ("engage a different box", 0.12),
        ("APPROACH → CARRY_HOME", 0.12),
    ]),
    ("S4. Box detected near home (skip)", "#cfe2ff", [
        ("SEARCH FORWARD", 0.18),
        ("vision sees blob inside _home_skip_radius", 0.18),
        ("filter: 'at_home' — skip, stay in SEARCH", 0.18),
        ("sweep continues, X-far turn-back at x=2000", 0.20),
        ("LOOK_RIGHT → realign → FORWARD back", 0.26),
    ]),
    ("S5. Wall in CARRY_HOME path", "#fde2e4", [
        ("CARRY_HOME NORMAL (driving spline)", 0.18),
        ("lidar obstacle_ahead → v zeroed", 0.10),
        ("stuck-watcher: 30 mm in 3 s → trip", 0.12),
        ("REVERSING (200 mm / 6 s)", 0.14),
        ("CURLING LEFT (midpoint +250 mm)", 0.20),
        ("arrive home, RELEASE", 0.26),
    ]),
    ("S6. Persistent stuck — release in place", "#f8d7da", [
        ("CARRY_HOME NORMAL", 0.14),
        ("stuck #1 → REVERSE → CURL LEFT", 0.22),
        ("stuck #2 → REVERSE → CURL RIGHT", 0.22),
        ("stuck again (attempts = max)", 0.10),
        ("log + RELEASE in place", 0.14),
        ("TURN_AROUND → SEARCH (no SAFE_STOP)", 0.18),
    ]),
    ("S7. GOTO with halt-and-localize", "#cfe2ff", [
        ("DONE", 0.04), ("go_to(x,y,θ) → GOTO", 0.06),
        ("plan spline (first tick)", 0.06),
        ("PATH_FOLLOW to PP.has_arrived", 0.34),
        ("halt (v=0; σ_xy → 30 mm / 700 ms)", 0.16),
        ("Mahalanobis² ≤ 5.99 (log only)", 0.10),
        ("→ _post_goto_state = DONE", 0.24),
    ]),
    ("S8. abort() during APPROACH", "#f8d7da", [
        ("SEARCH", 0.16),
        ("APPROACH CHASE", 0.20),
        ("abort() called from main", 0.04),
        ("_controller.abort() + SAFE_STOP", 0.20),
        ("motors stopped (Command::STOP)", 0.40),
    ]),
]


def scenarios_timeline():
    rows = len(SCENARIOS)
    fig, ax = plt.subplots(figsize=(16, 0.95 * rows + 1.2))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, rows + 1)
    ax.axis("off")
    ax.set_title("Mission scenarios — what the robot does and why",
                 fontsize=15, weight="bold")

    bar_left = 4.2
    bar_w    = 11.4
    for i, (title, color, segs) in enumerate(SCENARIOS):
        y = rows - i
        ax.text(0.1, y, title, ha="left", va="center", fontsize=10,
                weight="bold")
        x = bar_left
        for j, (lbl, frac) in enumerate(segs):
            w = bar_w * frac
            shade = color if j % 2 == 0 else _lighten(color, 0.55)
            ax.add_patch(Rectangle((x, y - 0.32), w, 0.64,
                                   facecolor=shade, edgecolor="black", lw=0.6))
            if w > 0.7:
                ax.text(x + w / 2, y, lbl, ha="center", va="center",
                        fontsize=7.0)
            x += w

    ax.text(8.0, 0.25,
            "Time axis is illustrative; segment widths show relative duration, not real seconds. "
            "See scenarios.md for the per-tick rationale.",
            ha="center", fontsize=8.5, style="italic", color="#444")
    save(fig, "scenarios.png")


def _lighten(hex_color, amount):
    """amount in [0, 1]; 0 = unchanged, 1 = white."""
    h = hex_color.lstrip("#")
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    r = int(r + (255 - r) * amount)
    g = int(g + (255 - g) * amount)
    b = int(b + (255 - b) * amount)
    return f"#{r:02x}{g:02x}{b:02x}"


# ---------- 8. Lock topology --------------------------------------------------
def lock_topology():
    fig, ax = plt.subplots(figsize=(12, 7))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 7)
    ax.axis("off")
    ax.set_title("Mutex / atomic protection map",
                 fontsize=14, weight="bold")

    items = [
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
         "has_target,\nbearing/dist/area (locked);\ncaptured/lost/grabbable (atomic)",
         "loop_vision_tracking",
         "loop_control (StateMachine.step)",
         "lock + atomics"),
        (2.0, 2.5, "StateMachine",
         "_state (atomic),\n_goto_*, _home_*,\n_approach_attempts,\n_post_goto_state",
         "loop_control (step),\nany thread (start/go_to/abort)",
         "loop_control + dashboard snap",
         "_m + CAS guard"),
        (6.0, 2.5, "PurePursuitController",
         "_path, _mode, _arrived",
         "StateMachine (set_path/abort)",
         "loop_control (compute)",
         "_m"),
        (10.0, 2.5, "app::* atomics",
         "shutdown, lidar_ready,\nlidar_stop, obstacle_ahead,\nobstacle_{left,right}_mm,\nrelocalizing, v_cmd_last_mm_s",
         "many",
         "many",
         "std::atomic"),
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
                ha="center", va="center", fontsize=7.2)
        ax.text(x, y - 0.30, f"W: {w}", ha="center", va="center", fontsize=7.2)
        ax.text(x, y - 0.65, f"R: {r}", ha="center", va="center", fontsize=7.2)
        ax.text(x, y - 0.95, f"lock: {lk}", ha="center", va="center",
                fontsize=7.2, style="italic")
    ax.text(6.0, 0.25,
            "obstacle_ahead is now consumed by core.cpp's gate (was the dead-flag bug); "
            "v_cmd_last_mm_s lets the cox loop correlate intended motion with measured motion.",
            ha="center", color="#444", fontsize=8.5, style="italic")
    save(fig, "lock_topology.png")


# ---------- 9. Per-tick control sequence -------------------------------------
def control_tick():
    fig, ax = plt.subplots(figsize=(11, 8))
    ax.set_xlim(0, 11)
    ax.set_ylim(0, 12)
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
        (7, "StateMachine.step(p, cov, dash)\n  reads TrackerResult.snapshot\n  emits Command (STOP / DIRECT / PATH_FOLLOW)"),
        (8, "switch on Command.mode:\n  PATH_FOLLOW: pp.compute(p, v_cmd, w_cmd)\n  DIRECT: use cmd.v_mm_s, cmd.omega_rad_s\n  STOP: zero both"),
        (9, "if app::relocalizing: v_cmd = w_cmd = 0"),
        (10, "if app::obstacle_ahead && v_cmd > 0\n  && !sm.obstacle_gate_suppressed(): v_cmd = 0"),
        (11, "motor.set_velocity(v_cmd, w_cmd)"),
        (12, "app::v_cmd_last_mm_s = v_cmd  (push-detect helper)"),
        (13, "publish dash + telemetry (10 Hz gated)"),
    ]
    y = 11.5
    for i, (n, t) in enumerate(steps):
        h = 0.85 if "\n" not in t else 1.05 if t.count("\n") == 2 else 1.25
        ax.add_patch(FancyBboxPatch((1.0, y - h / 2), 9.0, h,
                                    boxstyle="round,pad=0.05",
                                    facecolor="#cfe2ff" if n in (7, 8, 10) else "#e9ecef",
                                    edgecolor="black", lw=1.0))
        ax.text(1.4, y, f"{n}.", fontsize=10, weight="bold",
                ha="left", va="center")
        ax.text(2.0, y, t, fontsize=9, ha="left", va="center")
        y -= h + 0.05
        if i < len(steps) - 1:
            ax.add_patch(FancyArrowPatch((5.5, y + h / 2 + 0.05),
                                         (5.5, y + h / 2 - 0.05),
                                         arrowstyle="->",
                                         mutation_scale=10,
                                         color="black", lw=0.8))

    ax.text(5.5, 0.1,
            "Steps 7/8 produce intent; step 10 vetoes forward motion on a "
            "lidar obstacle (unless the state suppresses the gate).",
            ha="center", color="#444", fontsize=8.5, style="italic")
    save(fig, "control_tick.png")


def main():
    thread_topology()
    data_flow()
    state_machine()
    search_substates()
    approach_substates()
    carry_substates()
    scenarios_timeline()
    lock_topology()
    control_tick()
    print("All diagrams generated.")


if __name__ == "__main__":
    main()
