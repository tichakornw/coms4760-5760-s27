"""The parking street: a bicycle-model car reverses into the 6.0 m gap between two parked cars.

The goal is 0.5 m from the rear car and 1.5 m from the front one; the maneuver ends 0.37 m and
1.56 m (see PARK_CONTROLS)."""
from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field

from shapely import Polygon, box

Pose = tuple[float, float, float]  # rear-axle x, y, heading θ in radians
Control = tuple[float, float, float]  # (v, δ, duration)
WHEEL = (0.6, 0.25)         # a drawn wheel's length and width in metres
WHEEL_HALF_TRACK = 0.75     # drawn wheels this far from the car's centre line


def _wrap(a: float) -> float:
    """Angle in (-π, π]."""
    a = math.fmod(a + math.pi, 2 * math.pi)
    return (a + 2 * math.pi if a <= 0 else a) - math.pi


@dataclass(frozen=True)
class Car:
    length: float = 4.0
    width: float = 1.8
    wheelbase: float = 2.5
    rear_overhang: float = 0.5
    max_steer: float = 0.6

    @property
    def centre_ahead(self) -> float:
        """How far the body's centre is ahead of the rear axle."""
        return self.length / 2 - self.rear_overhang

    def centre(self, pose: Pose) -> tuple[float, float]:
        """The body's centre point at a pose."""
        x, y, th = pose
        return (x + self.centre_ahead * math.cos(th), y + self.centre_ahead * math.sin(th))

    def footprint(self, pose: Pose) -> Polygon:
        """The body rectangle: rear_overhang behind the rear axle, length - rear_overhang ahead."""
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        back, front, half = -self.rear_overhang, self.length - self.rear_overhang, self.width / 2
        return Polygon([(x + u * c - w * s, y + u * s + w * c)
                        for u, w in ((back, -half), (front, -half), (front, half), (back, half))])

    def wheels(self, pose: Pose, delta: float = 0.0) -> list[list[tuple[float, float]]]:
        """The four wheels as rectangles (corner lists) for drawing: WHEEL long by WHEEL wide,
        WHEEL_HALF_TRACK either side of the centre line at the rear axle and at the front axle,
        the front pair turned by δ."""
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        out = []
        for u in (0.0, self.wheelbase):
            for w in (-WHEEL_HALF_TRACK, WHEEL_HALF_TRACK):
                cx, cy = x + u * c - w * s, y + u * s + w * c
                a = th + (delta if u > 0 else 0.0)
                ca, sa = math.cos(a), math.sin(a)
                hl, hw = WHEEL[0] / 2, WHEEL[1] / 2
                out.append([(cx + p * ca - q * sa, cy + p * sa + q * ca)
                            for p, q in ((-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw))])
        return out

    def step(self, pose: Pose, v: float, delta: float, dt: float) -> Pose:
        """Kinematic bicycle, one Euler step; δ clamped to ±max_steer."""
        x, y, th = pose
        delta = max(-self.max_steer, min(self.max_steer, delta))
        return (x + v * math.cos(th) * dt, y + v * math.sin(th) * dt,
                th + v / self.wheelbase * math.tan(delta) * dt)


@dataclass(frozen=True)
class ParkingWorld:
    bounds: tuple[float, float, float, float] = (0, 0, 15, 5)
    curb_y: float = 0.0
    parked: tuple[Polygon, ...] = (box(0.5, 0.1, 4.5, 1.9), box(10.5, 0.1, 14.5, 1.9))
    car: Car = field(default_factory=Car)
    start: Pose = (11.5, 3.2, 0.0)
    goal: Pose = (5.5, 1.0, 0.0)
    goal_tol: tuple[float, float, float] = (0.3, 0.3, 0.1)

    def collides(self, pose: Pose) -> bool:
        """The footprint touches a parked car, crosses the curb line y = curb_y, or leaves the bounds."""
        fp = self.car.footprint(pose)
        xmin, ymin, xmax, ymax = fp.bounds
        bx0, by0, bx1, by1 = self.bounds
        if ymin < self.curb_y or xmin < bx0 or ymin < by0 or xmax > bx1 or ymax > by1:
            return True
        return any(fp.intersects(p) for p in self.parked)

    def at_goal(self, pose: Pose) -> bool:
        tx, ty, tth = self.goal_tol
        return (abs(pose[0] - self.goal[0]) <= tx and abs(pose[1] - self.goal[1]) <= ty
                and abs(_wrap(pose[2] - self.goal[2])) <= tth)

    def simulate(self, pose: Pose, controls: Sequence[Control], dt: float = 0.05) -> list[Pose]:
        """Every pose from `pose` on: each control (v, δ, duration) runs round(duration / dt) steps."""
        poses = [pose]
        for v, delta, duration in controls:
            for _ in range(round(duration / dt)):
                poses.append(self.car.step(poses[-1], v, delta, dt))
        return poses

    def park_maneuver(self) -> list[Control]:
        """Reverse in: an arc steering toward the curb, an arc the other way, then straight (possibly empty).
        PARK_CONTROLS for the default street, else searched."""
        return list(PARK_CONTROLS if self == PARKING_WORLD else search_park_controls(self))

    def slide_in_path(self) -> list[Pose]:
        """The move a point could make but the car cannot: a pure sideways translation at the
        goal's heading from (goal x, start y) straight down to the goal pose, above the gap,
        sampled every 0.1 of distance."""
        gx, gy, th = self.goal
        y0 = self.start[1]
        n = max(1, math.ceil(abs(y0 - gy) / 0.1 - 1e-9))
        return [(gx, y0 + k / n * (gy - y0), th) for k in range(n + 1)]


PARKING_WORLD = ParkingWorld()


def clearances(world: ParkingWorld, pose: Pose) -> tuple[float, float]:
    """(metres from the car's footprint to the parked car behind it, to the one ahead):
    the gaps between the footprint's x extent and the two parked cars' boxes."""
    x0, _, x1, _ = world.car.footprint(pose).bounds
    return (x0 - world.parked[0].bounds[2], world.parked[1].bounds[0] - x1)


def pose_error(world: ParkingWorld, pose: Pose) -> float:
    """End-pose error to the goal as a fraction of the tolerance: the largest of
    |Δx| / tol_x, |Δy| / tol_y, |Δθ| / tol_θ (at most 1 when at_goal holds)."""
    tx, ty, tth = world.goal_tol
    return max(abs(pose[0] - world.goal[0]) / tx, abs(pose[1] - world.goal[1]) / ty,
               abs(_wrap(pose[2] - world.goal[2])) / tth)

# Search grid for the reverse-in maneuver, v = -1: (δ1, t1, δ2 = -δ1, t2, t3), then a local
# refinement (refine_park_controls) of the first hit.
_STEERS = (-0.6, -0.5, -0.4)
_ARC_TIMES = tuple(round(0.5 + 0.1 * k, 1) for k in range(36))  # 0.5 .. 4.0
_STRAIGHT_TIMES = tuple(round(0.1 * k, 1) for k in range(16))  # 0 .. 1.5
REFINE_WINDOW, REFINE_STEP = 0.5, 0.05        # durations within ±0.5 s on a 0.05 s grid
REFINE_STEER_WINDOW, REFINE_STEER_STEP = 0.1, 0.05  # steering angles within ±0.1 in 0.05 steps


def search_park_controls(world: ParkingWorld = PARKING_WORLD, dt: float = 0.05,
                         steers: Sequence[float] = _STEERS, arc_times: Sequence[float] = _ARC_TIMES,
                         straight_times: Sequence[float] = _STRAIGHT_TIMES) -> tuple[Control, ...]:
    """The coarse search (`coarse_park_controls`) refined by `refine_park_controls`."""
    return refine_park_controls(world, coarse_park_controls(world, dt, steers, arc_times,
                                                            straight_times), dt)


def _steps(world: ParkingWorld, pose: Pose, v: float, delta: float, n: int, dt: float) -> list[Pose]:
    """Up to n collision-free Euler steps from pose; stops at the first collision."""
    out = []
    for _ in range(n):
        pose = world.car.step(pose, v, delta, dt)
        if world.collides(pose):
            break
        out.append(pose)
    return out


def refine_park_controls(world: ParkingWorld, coarse: Sequence[Control],
                         dt: float = 0.05) -> tuple[Control, ...]:
    """Around a coarse reverse-in maneuver ((v, δ1, t1), (v, δ2, t2), (v, 0, t3)): every
    duration within ±REFINE_WINDOW on a REFINE_STEP grid (t3 not below 0) and each steering
    angle within ±REFINE_STEER_WINDOW in REFINE_STEER_STEP steps; of the collision-free
    combinations that end at_goal, the one with the smallest `pose_error` (ties: first in
    grid order, δ1, t1, δ2, t2, t3 ascending)."""
    (v, d1, t1), (_, d2, t2), (_, _, t3) = coarse

    def times(t: float) -> list[float]:
        lo = max(0.0, t - REFINE_WINDOW)
        return [round(lo + REFINE_STEP * k, 2) for k in range(int(round(2 * REFINE_WINDOW / REFINE_STEP)) + 1)
                if lo + REFINE_STEP * k <= t + REFINE_WINDOW + 1e-9]

    def steers(d: float) -> list[float]:
        n = int(round(REFINE_STEER_WINDOW / REFINE_STEER_STEP))
        return [round(d + REFINE_STEER_STEP * k, 2) for k in range(-n, n + 1)]

    T1, T2, T3 = times(t1), times(t2), times(t3)
    best = None
    for s1 in steers(d1):
        p1 = _steps(world, world.start, v, s1, round(max(T1) / dt), dt)
        for a in T1:
            n1 = round(a / dt)
            if n1 > len(p1):
                break
            q1 = world.start if n1 == 0 else p1[n1 - 1]
            for s2 in steers(d2):
                p2 = _steps(world, q1, v, s2, round(max(T2) / dt), dt)
                for b in T2:
                    n2 = round(b / dt)
                    if n2 > len(p2):
                        break
                    q2 = q1 if n2 == 0 else p2[n2 - 1]
                    p3 = _steps(world, q2, v, 0.0, round(max(T3) / dt), dt)
                    for c in T3:
                        n3 = round(c / dt)
                        if n3 > len(p3):
                            break
                        q3 = q2 if n3 == 0 else p3[n3 - 1]
                        if world.at_goal(q3):
                            e = pose_error(world, q3)
                            if best is None or e < best[0]:
                                best = (e, ((v, s1, a), (v, s2, b), (v, 0.0, c)))
    if best is None:
        raise ValueError("no refinement of the coarse maneuver reaches the goal")
    return best[1]


def coarse_park_controls(world: ParkingWorld = PARKING_WORLD, dt: float = 0.05,
                         steers: Sequence[float] = _STEERS, arc_times: Sequence[float] = _ARC_TIMES,
                         straight_times: Sequence[float] = _STRAIGHT_TIMES) -> tuple[Control, ...]:
    """The first (δ1, t1, t2, t3) in grid order (δ1, then t1, t2, t3 ascending) whose controls
    ((-1, δ1, t1), (-1, -δ1, t2), (-1, 0, t3)) simulated at dt keep the car collision-free and
    end at_goal. Paths are stepped incrementally (the same Euler steps as `simulate`) and a
    prefix that collides prunes everything after it."""
    v = -1.0
    car = world.car
    n = {t: round(t / dt) for t in (*arc_times, *straight_times)}
    for d1 in steers:
        p1, k1 = world.start, 0
        for t1 in arc_times:
            ok = True
            while k1 < n[t1]:
                p1, k1 = car.step(p1, v, d1, dt), k1 + 1
                if world.collides(p1):
                    ok = False
                    break
            if not ok:
                break
            p2, k2 = p1, 0
            for t2 in arc_times:
                ok = True
                while k2 < n[t2]:
                    p2, k2 = car.step(p2, v, -d1, dt), k2 + 1
                    if world.collides(p2):
                        ok = False
                        break
                if not ok:
                    break
                p3, k3 = p2, 0
                for t3 in straight_times:
                    ok = True
                    while k3 < n[t3]:
                        p3, k3 = car.step(p3, v, 0.0, dt), k3 + 1
                        if world.collides(p3):
                            ok = False
                            break
                    if not ok:
                        break
                    if world.at_goal(p3):
                        return ((v, d1, t1), (v, -d1, t2), (v, 0.0, t3))
    raise ValueError("no reverse-in maneuver on this search grid")


# search_park_controls() on PARKING_WORLD (tests/test_parking_world.py reproduces it): the
# coarse search gives (-1, -0.4, 3.6), (-1, 0.4, 3.1), (-1, 0, 0), ending at (5.24, 1.10) with
# a heading of 4.8 degrees and only 0.17 m behind; the refinement ends at (5.40, 1.11) with a
# heading of 2.4 degrees, 0.37 m behind and 1.56 m ahead. The straight segment is empty.
PARK_CONTROLS: tuple[Control, ...] = ((-1.0, -0.4, 3.8), (-1.0, 0.5, 2.75), (-1.0, 0.0, 0.0))
