"""
Corner-route demo: plan the shortest route through obstacle corners once (visibility graph +
Dijkstra), then drive it leg by leg with an Autopilot-style controller.

Run:  python3 tools/corner_route_demo/corner_route_demo.py
Out:  tools/corner_route_demo/corner_route_demo.png  (+ stats printed to stdout)

Units: metres, seconds. Blue-alliance coordinates. Obstacles are the hub and tower from
FieldConstants.kt plus one ILLUSTRATIVE block (not real field geometry) that makes a narrow gap.
The robot is a point with a first-order velocity lag; rotation, friction and noise are ignored.
"""

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# ----------------------------------------------------------------------------- parameters
FIELD_X, FIELD_Y = 16.54, 8.07
ROBOT_RADIUS = 0.45  # bumper half-width [m]
CLEARANCE = 0.5  # line-of-sight margin: robot radius + 5 cm [m]
CORNER_OFFSET = 0.7  # corner waypoints sit this far out from the rectangle, on both axes [m]
SWITCH_DISTANCE = 0.3  # move on to the next leg this close to a corner [m]

V_MAX, ACCEL, JERK = 4.0, 5.5, 300.0  # Autopilot constraints (AlignToPose.kt) + top speed
LATERAL_ACCEL = 5.0  # sideways grip used for the corner speed [m/s^2]
DT, TAU = 0.02, 0.04  # loop period, drivetrain velocity lag [s]

HUB = (4.62041, 4.03463)
HUB_HALF = 1.194 / 2
RECTS = [  # (xmin, ymin, xmax, ymax)
    (HUB[0] - HUB_HALF, HUB[1] - HUB_HALF, HUB[0] + HUB_HALF, HUB[1] + HUB_HALF),  # hub
    (0.0, 3.113, 1.060, 4.363),  # tower (CLIMB_RECTANGLE)
    (4.0, 0.0, 5.2, 2.0),  # ILLUSTRATIVE block
]
GOAL = np.array([2.6, 4.03])
STARTS = [(7.6, 4.03), (7.2, 1.2), (6.8, 7.2), (1.2, 7.4), (7.5, 6.0), (6.0, 1.0), (8.0, 2.5),
          (5.0, 6.5)]


# ----------------------------------------------------------------------------- planner
def segment_hits_box(p, q, box, grow):
    """Liang-Barsky clipping: does segment p->q pass through `box` grown by `grow`?"""
    x0, y0, x1, y1 = box[0] - grow, box[1] - grow, box[2] + grow, box[3] + grow
    d = q - p
    t_enter, t_exit = 0.0, 1.0
    for denom, num in ((-d[0], p[0] - x0), (d[0], x1 - p[0]), (-d[1], p[1] - y0), (d[1], y1 - p[1])):
        if denom == 0:
            if num < 0:
                return False  # parallel to this edge and outside it
            continue
        t = num / denom
        if denom < 0:
            t_enter = max(t_enter, t)
        else:
            t_exit = min(t_exit, t)
        if t_enter > t_exit:
            return False
    return True


def is_clear(p, q):
    return not any(segment_hits_box(p, q, box, CLEARANCE) for box in RECTS)


def corner_waypoints():
    """Each rectangle's corners pushed out by CORNER_OFFSET, kept if inside the field and free."""
    out = []
    for x0, y0, x1, y1 in RECTS:
        for c in ((x0 - CORNER_OFFSET, y0 - CORNER_OFFSET), (x1 + CORNER_OFFSET, y0 - CORNER_OFFSET),
                  (x0 - CORNER_OFFSET, y1 + CORNER_OFFSET), (x1 + CORNER_OFFSET, y1 + CORNER_OFFSET)):
            c = np.array(c)
            in_field = CLEARANCE < c[0] < FIELD_X - CLEARANCE and CLEARANCE < c[1] < FIELD_Y - CLEARANCE
            free = not any(b[0] - CLEARANCE <= c[0] <= b[2] + CLEARANCE and
                           b[1] - CLEARANCE <= c[1] <= b[3] + CLEARANCE for b in RECTS)
            if in_field and free:
                out.append(c)
    return out


def plan(start, goal):
    """Shortest route start -> goal through visible corners (Dijkstra). Returns [start, ..., goal]."""
    nodes = [np.asarray(start, float), np.asarray(goal, float)] + corner_waypoints()
    dist = [math.inf] * len(nodes)
    prev = [None] * len(nodes)
    done = [False] * len(nodes)
    dist[0] = 0.0
    for _ in nodes:
        i = min((k for k in range(len(nodes)) if not done[k]), key=lambda k: dist[k])
        if dist[i] == math.inf or i == 1:
            break
        done[i] = True
        for j in range(len(nodes)):
            if not done[j] and is_clear(nodes[i], nodes[j]):
                d = dist[i] + np.linalg.norm(nodes[j] - nodes[i])
                if d < dist[j]:
                    dist[j], prev[j] = d, i
    if dist[1] == math.inf:
        return None
    route, i = [], 1
    while i is not None:
        route.append(nodes[i])
        i = prev[i]
    return route[::-1]


def corner_speed(before, corner, after):
    """Speed to carry through a corner: the robot rounds it on an arc of radius ~CORNER_OFFSET."""
    a, b = corner - before, after - corner
    turn = math.acos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))
    return min(V_MAX, math.sqrt(LATERAL_ACCEL * CORNER_OFFSET / max(math.tan(turn / 2), 1e-3)))


# ----------------------------------------------------------------------------- Autopilot model
X0 = ACCEL ** 3 / (18 * JERK ** 2)
V0 = (4.5 * X0 ** 2 * JERK) ** (1 / 3)


def profile(d):
    """APConstraints.calculateMaxVelocity: fastest speed that can still stop in distance d."""
    return math.sqrt(V0 ** 2 + 2 * ACCEL * (d - X0)) if d > X0 else (4.5 * d * d * JERK) ** (1 / 3)


def autopilot_step(p, v, target, end_speed):
    """Autopilot.calculate in beeline mode: speed = profile + end speed, and correct() keeps only
    the velocity component toward the target, raising it by at most ACCEL*DT."""
    d = np.linalg.norm(target - p)
    if d < 1e-9:
        return np.zeros(2)
    u = (target - p) / d
    goal_speed = min(V_MAX, profile(d) + end_speed)
    along = float(v @ u)
    return u * min(goal_speed, along + ACCEL * DT)


def bumper_clearance(p):
    boxes = [math.hypot(max(b[0] - p[0], 0, p[0] - b[2]), max(b[1] - p[1], 0, p[1] - b[3])) for b in RECTS]
    return min(boxes + [p[0], FIELD_X - p[0], p[1], FIELD_Y - p[1]]) - ROBOT_RADIUS


def simulate(start, t_max=10.0):
    route = plan(start, GOAL)
    legs = route[1:]
    ends = [corner_speed(route[i], route[i + 1], route[i + 2]) for i in range(len(route) - 2)] + [0.0]
    p, v, leg = np.array(start, float), np.zeros(2), 0
    log = {"t": [], "p": [p.copy()], "speed": [], "clear": []}
    for k in range(int(t_max / DT)):
        if leg < len(legs) - 1 and np.linalg.norm(legs[leg] - p) < SWITCH_DISTANCE:
            leg += 1
        command = autopilot_step(p, v, legs[leg], ends[leg])
        v += (command - v) * (DT / TAU)
        p = p + v * DT
        log["t"].append(k * DT)
        log["p"].append(p.copy())
        log["speed"].append(np.linalg.norm(v))
        log["clear"].append(bumper_clearance(p))
        if leg == len(legs) - 1 and np.linalg.norm(GOAL - p) < 0.01 and np.linalg.norm(v) < 0.02:
            break
    out = {k: np.array(val) for k, val in log.items()}
    out["route"], out["ends"] = route, ends
    return out


# ----------------------------------------------------------------------------- run + plot
def main():
    runs = [simulate(s) for s in STARTS]
    for s, r in zip(STARTS, runs):
        print(f"start {s}: {len(r['route']) - 2} corner(s), {r['t'][-1]:.2f} s, "
              f"final error {np.linalg.norm(GOAL - r['p'][-1]) * 100:.2f} cm, "
              f"min bumper clearance {r['clear'].min() * 100:.1f} cm")

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(16, 7.5), gridspec_kw={"width_ratios": [1.3, 1]})
    for b in RECTS:
        ax.add_patch(plt.Rectangle((b[0], b[1]), b[2] - b[0], b[3] - b[1], color="black"))
        ax.add_patch(plt.Rectangle((b[0] - CLEARANCE, b[1] - CLEARANCE), b[2] - b[0] + 2 * CLEARANCE,
                                   b[3] - b[1] + 2 * CLEARANCE, fill=False, ec="red", ls="--", lw=1))
    corners = np.array(corner_waypoints())
    ax.plot(corners[:, 0], corners[:, 1], "^", color="green", ms=9, label="corner waypoints")
    colors = plt.cm.tab10(np.linspace(0, 1, len(runs)))
    for r, c in zip(runs, colors):
        route = np.array(r["route"])
        ax.plot(route[:, 0], route[:, 1], ":", color=c, lw=1.2)
        ax.plot(r["p"][:, 0], r["p"][:, 1], color=c, lw=2)
        ax.plot(*r["p"][0], "o", color=c, mec="black")
    ax.plot(*GOAL, "*", color="gold", ms=20, mec="black", label="goal")
    ax.plot([], [], ":", color="gray", label="planned route")
    ax.plot([], [], color="gray", lw=2, label="driven path")
    ax.plot([], [], "r--", lw=1, label="line-of-sight margin")
    ax.text(4.6, 1.0, "illustrative\nblock", color="white", ha="center", va="center", fontsize=8)
    ax.set_xlim(0, 9.0)
    ax.set_ylim(0, FIELD_Y)
    ax.set_aspect("equal")
    ax.set_title("Plan once through obstacle corners, drive each leg with Autopilot")
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)

    for r, c, s in zip(runs, colors, STARTS):
        ax2.plot(r["t"], r["speed"], color=c, label=f"from {s}")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("speed [m/s]")
    ax2.set_title("Speed: slows only as much as each corner needs")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    out = Path(__file__).with_suffix(".png")
    fig.savefig(out, dpi=100)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
