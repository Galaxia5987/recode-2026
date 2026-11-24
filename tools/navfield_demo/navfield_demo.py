"""
Navigation-field demo: hill-cost grid -> Fast Marching cost-to-go T -> drive down -grad(T)
-> hand over to an Autopilot-style terminal approach.

Compared against a classic artificial potential field (APF), which gets stuck behind the hub.

Run:  python3 tools/navfield_demo/navfield_demo.py
Out:  tools/navfield_demo/navfield_demo.png  (+ stats printed to stdout)

Units: metres, seconds. Blue-alliance coordinates. Obstacles are the hub and tower from
FieldConstants.kt plus one ILLUSTRATIVE block (not real field geometry) that makes a
narrow gap, to show that the robot drives through the middle of it.
"""

import heapq
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# ----------------------------------------------------------------------------- parameters
FIELD_X, FIELD_Y = 16.54, 8.07
CELL = 0.05  # grid resolution [m]

ROBOT_RADIUS = 0.45  # bumper half-width [m]
SAFETY = 0.05  # extra margin on top of the robot radius [m]
INFLATE = ROBOT_RADIUS + SAFETY

HILL_RANGE = 0.5  # d0: how far from an (inflated) obstacle the hill reaches [m]
HILL_WEIGHT = 3.0  # w: cost multiplier right at the inflated edge

# Autopilot constraints (same accel/jerk as lib/align/AlignToPose.kt, plus a speed cap)
V_MAX, A_MAX, JERK = 4.5, 5.5, 300.0
A_LAT = 5.0  # max sideways accel used by the polar rate limiter [m/s^2]
KP_TERMINAL = 5.0  # linear cap near the goal: v <= kP * d  [1/s]
HANDOVER_T = 0.6  # switch from nav field to Autopilot when T < this [m]

DT = 0.02
TAU = 0.04  # drivetrain velocity lag (first order) [s]
LATENCY_CYCLES = 2  # the controller sees the pose from this many cycles ago (40 ms)

HUB_SIDE = 1.194
HUB_BLUE = (4.62041, 4.03463)
RECTS = [  # (xmin, ymin, xmax, ymax)
    (HUB_BLUE[0] - HUB_SIDE / 2, HUB_BLUE[1] - HUB_SIDE / 2,
     HUB_BLUE[0] + HUB_SIDE / 2, HUB_BLUE[1] + HUB_SIDE / 2),  # blue hub
    (0.0, 3.113, 1.060, 4.363),  # blue tower (CLIMB_RECTANGLE)
    (4.0, 0.0, 5.2, 2.0),  # ILLUSTRATIVE block -> narrow gap below the hub
]
RECTS += [(FIELD_X - r[2], r[1], FIELD_X - r[0], r[3]) for r in RECTS[:2]]  # red hub + tower

GOAL = np.array([2.6, 4.03])
STARTS = [np.array(p) for p in [(7.6, 4.03), (7.2, 1.2), (6.8, 7.2), (1.2, 7.4)]]


# ----------------------------------------------------------------------------- geometry
def dist_to_obstacles(x, y):
    """Distance from (x, y) to the nearest raw obstacle surface or field wall (vectorised)."""
    d = np.minimum.reduce([x, FIELD_X - x, y, FIELD_Y - y])
    for xmin, ymin, xmax, ymax in RECTS:
        dx = np.maximum.reduce([xmin - x, np.zeros_like(x), x - xmax])
        dy = np.maximum.reduce([ymin - y, np.zeros_like(y), y - ymax])
        d = np.minimum(d, np.hypot(dx, dy))
    return d


def hill(d):
    """Quadratic bowl: 1 at the inflated edge, 0 at HILL_RANGE, smooth (zero slope) at HILL_RANGE."""
    return np.clip(1.0 - d / HILL_RANGE, 0.0, 1.0) ** 2


# ----------------------------------------------------------------------------- grid + FMM
nx, ny = int(round(FIELD_X / CELL)) + 1, int(round(FIELD_Y / CELL)) + 1
gx, gy = np.meshgrid(np.arange(nx) * CELL, np.arange(ny) * CELL, indexing="ij")
clearance = dist_to_obstacles(gx, gy) - INFLATE  # d: distance past the inflated edge
COST = np.where(clearance <= 0, np.inf, 1.0 + HILL_WEIGHT * hill(clearance))


def fast_marching(cost, goal):
    """First-order Fast Marching: solves |grad T| = cost outward from the goal."""
    T = np.full(cost.shape, np.inf)
    frozen = np.zeros(cost.shape, bool)
    gi, gj = int(round(goal[0] / CELL)), int(round(goal[1] / CELL))
    T[gi, gj] = 0.0
    heap = [(0.0, gi, gj)]
    while heap:
        t, i, j = heapq.heappop(heap)
        if frozen[i, j]:
            continue
        frozen[i, j] = True
        for ni, nj in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if not (0 <= ni < nx and 0 <= nj < ny) or frozen[ni, nj] or not np.isfinite(cost[ni, nj]):
                continue
            a = min(T[ni - 1, nj] if ni > 0 else np.inf, T[ni + 1, nj] if ni < nx - 1 else np.inf)
            b = min(T[ni, nj - 1] if nj > 0 else np.inf, T[ni, nj + 1] if nj < ny - 1 else np.inf)
            hc = CELL * cost[ni, nj]
            if abs(a - b) >= hc:  # only one upwind neighbour contributes
                cand = min(a, b) + hc
            else:  # both contribute: solve (T-a)^2 + (T-b)^2 = hc^2
                cand = 0.5 * (a + b + math.sqrt(2 * hc * hc - (a - b) ** 2))
            if cand < T[ni, nj]:
                T[ni, nj] = cand
                heapq.heappush(heap, (cand, ni, nj))
    return T


T_GRID = fast_marching(COST, GOAL)
# For interpolation, give obstacle cells a large finite value so the gradient points out of them.
T_FILLED = np.where(np.isfinite(T_GRID), T_GRID, np.nanmax(T_GRID[np.isfinite(T_GRID)]) + 5.0)


def T_at(p):
    """Bilinear interpolation of T at a point."""
    fx, fy = np.clip(p[0] / CELL, 0, nx - 1.001), np.clip(p[1] / CELL, 0, ny - 1.001)
    i, j = int(fx), int(fy)
    u, v = fx - i, fy - j
    return ((1 - u) * (1 - v) * T_FILLED[i, j] + u * (1 - v) * T_FILLED[i + 1, j]
            + (1 - u) * v * T_FILLED[i, j + 1] + u * v * T_FILLED[i + 1, j + 1])


def downhill(p):
    """Unit vector along -grad T (central differences on the interpolated field)."""
    h = CELL
    g = np.array([T_at(p + (h, 0)) - T_at(p - (h, 0)), T_at(p + (0, h)) - T_at(p - (0, h))]) / (2 * h)
    n = np.linalg.norm(g)
    return -g / n if n > 1e-9 else np.zeros(2)


# ----------------------------------------------------------------------------- Autopilot bits
X0 = A_MAX ** 3 / (18 * JERK ** 2)


def jerk_v(d):
    return (4.5 * d * d * JERK) ** (1 / 3)


V0 = jerk_v(X0)


def profile_speed(d, linear_cap=True):
    """APConstraints.calculateMaxVelocity, optionally capped by kP*d near the goal (fix A)."""
    v = math.sqrt(V0 * V0 + 2 * A_MAX * (d - X0)) if d > X0 else jerk_v(d)
    v = min(v, V_MAX)
    return min(v, KP_TERMINAL * d) if linear_cap else v


def limit_vector(v_cur, v_goal):
    """Clamp the change of the full 2D velocity vector to A_MAX*dt (cuts the chord in turns).
    Like Autopilot's correct() (min(goal, push(...))), braking is not limited: the speed profile
    already encodes the braking, so the result is never faster than the goal."""
    delta = v_goal - v_cur
    n = np.linalg.norm(delta)
    out = v_goal if n <= A_MAX * DT else v_cur + delta * (A_MAX * DT / n)
    s_out, s_goal = np.linalg.norm(out), np.linalg.norm(v_goal)
    return out * (s_goal / s_out) if s_out > s_goal else out


def limit_polar(v_cur, v_goal):
    """Limit acceleration to A_MAX and heading change to A_LAT/speed -> keeps speed in turns."""
    s, s_goal = np.linalg.norm(v_cur), np.linalg.norm(v_goal)
    if s < 0.3 or s_goal < 1e-6:  # slow or stopping: plain vector limit is fine
        return limit_vector(v_cur, v_goal)
    th, th_goal = math.atan2(v_cur[1], v_cur[0]), math.atan2(v_goal[1], v_goal[0])
    dth = (th_goal - th + math.pi) % (2 * math.pi) - math.pi
    max_dth = A_LAT / s * DT
    th += max(-max_dth, min(max_dth, dth))
    s_goal *= max(math.cos(dth), 0.0)  # pointing the wrong way -> slow down instead of orbiting
    s = min(s + A_MAX * DT, s_goal)  # accelerate at A_MAX, brake immediately (like Autopilot)
    return s * np.array([math.cos(th), math.sin(th)])


def corner_speed_limit(p, step=0.1, horizon=3.0):
    """Trace the downhill streamline ahead, estimate its curvature, and return the fastest speed
    from which the robot can still brake to sqrt(A_LAT / curvature) at every point ahead."""
    v_lim, q, s = V_MAX, p.copy(), 0.0
    d_prev = downhill(q)
    while s < horizon and T_at(q) > HANDOVER_T:
        q = q + step * d_prev
        s += step
        d_next = downhill(q)
        turn = math.acos(max(-1.0, min(1.0, float(d_prev @ d_next))))
        if turn > 1e-6:
            v_corner = math.sqrt(A_LAT * step / turn)  # curvature = turn / step
            v_lim = min(v_lim, math.sqrt(v_corner ** 2 + 2 * A_MAX * s))
        d_prev = d_next
    return v_lim


# ----------------------------------------------------------------------------- simulation
def simulate(start, mode="navfield", limiter=limit_polar, linear_cap=True, t_max=8.0,
             latency=LATENCY_CYCLES):
    """mode: 'navfield' (nav field + Autopilot handover) or 'apf' (classic potential field)."""
    p, v_actual, v_cmd = start.astype(float).copy(), np.zeros(2), np.zeros(2)
    log = {"t": [], "p": [], "speed": [], "clear": [], "dist": [], "phase": []}
    settled = 0
    history = [p.copy()] * (latency + 1)  # delayed pose measurements
    for k in range(int(t_max / DT)):
        p_true, p = p, history[0]  # control on the delayed measurement
        dist = np.linalg.norm(GOAL - p)
        if mode == "apf":
            goal_v = apf_velocity(p)
            phase = 0
        elif T_at(p) > HANDOVER_T:  # nav field: direction from the field, speed from T
            speed = min(profile_speed(T_at(p), linear_cap), corner_speed_limit(p))
            goal_v = downhill(p) * speed
            phase = 0
        else:  # Autopilot terminal (beeline) approach to the exact point
            goal_v = (GOAL - p) / max(dist, 1e-9) * profile_speed(dist, linear_cap)
            phase = 1
        # Like Autopilot's correct(): start from the last command, not the noisy measurement.
        v_cmd = limiter(v_cmd, goal_v)
        v_actual += (v_cmd - v_actual) * (DT / TAU) if TAU > DT else (v_cmd - v_actual)
        p = p_true + v_actual * DT
        history = history[1:] + [p.copy()]

        log["t"].append(k * DT)
        log["p"].append(p.copy())
        log["speed"].append(np.linalg.norm(v_actual))
        log["clear"].append(float(dist_to_obstacles(np.array(p[0]), np.array(p[1]))) - ROBOT_RADIUS)
        log["dist"].append(np.linalg.norm(GOAL - p))
        log["phase"].append(phase)
        # done = within 1 cm AND below 2 cm/s, debounced 5 cycles
        settled = settled + 1 if (log["dist"][-1] < 0.01 and log["speed"][-1] < 0.02) else 0
        if settled >= 5 and mode != "apf":
            break
    return {k: np.array(v) for k, v in log.items()}


def apf_velocity(p, k_att=1.5, eta=0.6, d0=1.0):
    """Classic Khatib APF: attractive + repulsive gradient, velocity = clamp(force)."""
    f = k_att * (GOAL - p)
    for xmin, ymin, xmax, ymax in RECTS:
        q = np.array([min(max(p[0], xmin), xmax), min(max(p[1], ymin), ymax)])
        diff = p - q
        d = max(np.linalg.norm(diff) - INFLATE, 0.05)
        if d < d0 and np.linalg.norm(diff) > 1e-9:
            f += eta * (1 / d - 1 / d0) / d ** 2 * diff / np.linalg.norm(diff)
    n = np.linalg.norm(f)
    return f if n <= V_MAX else f * V_MAX / n


# ----------------------------------------------------------------------------- run + plot
def fmt(p):
    return f"({p[0]:.1f}, {p[1]:.1f})"


def main():
    out = Path(__file__).with_suffix(".png")
    runs = [simulate(s) for s in STARTS]
    apf = simulate(STARTS[0], mode="apf", limiter=limit_vector)
    vec = simulate(STARTS[0], limiter=limit_vector)
    nocap = simulate(STARTS[0], linear_cap=False)
    apf_stuck = apf["dist"][-1] > 0.1

    print(f"grid {nx}x{ny} = {nx * ny} cells at {CELL} m, pose latency {LATENCY_CYCLES * DT * 1000:.0f} ms")
    for s, r in zip(STARTS, runs):
        print(f"nav field from {fmt(s)}: settled in {r['t'][-1]:.2f} s, final error {r['dist'][-1] * 100:.2f} cm, "
              f"min bumper clearance {r['clear'].min() * 100:.1f} cm")
    print(f"vector limiter from {fmt(STARTS[0])}: {vec['t'][-1]:.2f} s (polar: {runs[0]['t'][-1]:.2f} s)")
    tail = nocap["dist"][-50:]
    print(f"raw profile, no kP cap: still {tail.min() * 100:.2f}-{tail.max() * 100:.2f} cm from goal after "
          f"{nocap['t'][-1]:.0f} s (capped: {runs[0]['dist'][-1] * 100:.2f} cm)")
    print(f"classic APF from {fmt(STARTS[0])}: ends {apf['dist'][-1]:.2f} m from goal at "
          f"{apf['speed'][-1]:.3f} m/s" + (" -> stuck in a local minimum" if apf_stuck else ""))

    fig = plt.figure(figsize=(15, 12))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1], height_ratios=[1.15, 1])

    # 1. field: T heat map, -grad T arrows, trajectories
    ax = fig.add_subplot(gs[0, 0])
    Tm = np.ma.masked_invalid(T_GRID)
    im = ax.imshow(Tm.T, origin="lower", extent=(0, FIELD_X, 0, FIELD_Y), cmap="viridis", alpha=0.85)
    ax.contour(gx, gy, Tm, levels=np.arange(0, 30, 0.5), colors="white", linewidths=0.4, alpha=0.6)
    ax.contour(gx, gy, clearance, levels=[0], colors="red", linewidths=1, linestyles="--")
    for xmin, ymin, xmax, ymax in RECTS:
        ax.add_patch(plt.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, color="black"))
    step = 8
    qx, qy = gx[::step, ::step], gy[::step, ::step]
    U, V = np.zeros_like(qx), np.zeros_like(qy)
    for a in range(qx.shape[0]):
        for b in range(qx.shape[1]):
            if clearance[a * step, b * step] > 0:
                U[a, b], V[a, b] = downhill(np.array([qx[a, b], qy[a, b]]))
    ax.quiver(qx, qy, U, V, color="white", scale=45, width=0.002, alpha=0.8)
    for r in runs:
        ax.plot(r["p"][:, 0], r["p"][:, 1], color="orange", lw=2.2)
        ax.plot(*r["p"][0], "o", color="orange", mec="black")
    ax.plot(apf["p"][:, 0], apf["p"][:, 1], color="magenta", lw=2,
            label="classic APF" + (" (stuck)" if apf_stuck else ""))
    ax.plot(*apf["p"][-1], "x", color="magenta", ms=12, mew=3)
    ax.plot(*GOAL, "*", color="yellow", ms=18, mec="black", label="goal")
    ax.plot([], [], color="orange", lw=2, label="nav field → Autopilot handover")
    ax.plot([], [], "r--", lw=1, label="inflated edge (robot-centre limit)")
    ax.text(4.6, 1.0, "illustrative\nblock", color="white", ha="center", va="center", fontsize=8)
    ax.set_xlim(0, 9.0)
    ax.set_ylim(0, FIELD_Y)
    ax.set_aspect("equal")
    ax.set_title("Cost-to-go T (colour, 0.5 m contours) and drive direction −∇T (arrows)")
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02, label="T  [cost-weighted metres to goal]")

    # 2. speed over time: polar vs vector limiter
    ax = fig.add_subplot(gs[0, 1])
    ax.plot(runs[0]["t"], runs[0]["speed"], label="polar limiter (speed + heading)")
    ax.plot(vec["t"], vec["speed"], "--", label="vector limiter")
    if (runs[0]["phase"] == 1).any():
        ax.axvline(runs[0]["t"][np.argmax(runs[0]["phase"] == 1)], color="gray", ls=":",
                   label="handover to Autopilot")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("speed [m/s]")
    ax.set_title(f"Speed from {fmt(STARTS[0])} (around the hub)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # 3. hill function / cost
    ax = fig.add_subplot(gs[1, 0])
    d = np.linspace(0, 0.8, 200)
    ax.plot(d, 1 + HILL_WEIGHT * hill(d))
    ax.axvline(HILL_RANGE, color="gray", ls=":", label=f"d₀ = {HILL_RANGE:g} m")
    ax.set_xlabel("d = distance past the inflated edge [m]")
    ax.set_ylabel("cost per metre")
    ax.set_title(f"Hill: cost = 1 + {HILL_WEIGHT:g}·(1 − d/d₀)²")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # 4. final approach with latency: kP*d cap vs raw jerk profile
    ax = fig.add_subplot(gs[1, 1])
    for r, lab in ((runs[0], "profile capped by kP·d"), (nocap, "raw Autopilot profile (no cap)")):
        ax.semilogy(r["t"], np.maximum(r["dist"], 1e-5) * 100, label=lab)
    ax.axhline(1.0, color="gray", ls=":", label="1 cm tolerance")
    ax.set_xlim(runs[0]["t"][-1] - 1.5, nocap["t"][-1])
    ax.set_ylim(1e-2, 1e2)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("distance to goal [cm]")
    ax.set_title(f"Final approach with {LATENCY_CYCLES * DT * 1000:.0f} ms pose latency")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
