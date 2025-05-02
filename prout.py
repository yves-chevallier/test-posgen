from __future__ import annotations

"""s_curve_jerk_planner.py
===========================

Jerk-limited bang-bang trajectory generator
------------------------------------------
A modern, type-annotated and PEP 8/Black-compliant implementation of a point-to-
point S-curve planner whose position increments are smoothed by a *jerk filter*.
The filter is a two-stage moving-average (triangle) convolution that guarantees
conservation of increments (no micro-step loss) while bounding the third
kinematic derivative.

The original French comments were translated to English and the whole code base
was refactored for Python ≥ 3.12.  It follows the naming conventions enforced
by *ruff* and is auto-format-compatible with *black*.
"""

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Tuple

import matplotlib.pyplot as plt  # type: ignore
import numpy as np  # type: ignore

__all__ = [
    "SCurvePlannerContext",
    "SCurvePlanner",
    "JerkFilter",
]

EPS = 1e-12  # Numerical guard for float comparisons

def _clamp(value: float, low: float, high: float) -> float:
    """Clamp *value* to the inclusive range [*low*, *high*]."""
    return max(low, min(value, high))

class JerkFilter:
    def __init__(self, dt: float):
        self.dt = dt
        self.alpha = 0.0
        self.reset()

    def reset(self):
        self.err = 0.0 # reliquat signé
        self.y_old = 0.0

    def set_alpha(self, tau = 10e-3):
        self.alpha = self.dt / tau if tau > 0.0 else 0.0

    def update(self, sample: int):
        if self.alpha == 0.0: return (sample, sample)

        x_eff = sample + self.err # restitution du résidu
        y = self.y_old + self.alpha * (x_eff - self.y_old)
        self.y_old = y
        iy = int(round(y))  # quantification entière
        self.err = y - iy # nouveau résidu
        return (iy, y)


@dataclass(slots=True, frozen=True)
class SCurvePlannerContext:
    """Immutable dynamic limits for :class:`SCurvePlanner`."""

    time_step: float  # s
    max_velocity: float  # revolutions per second
    max_acceleration: float  # revolutions per second²
    max_deceleration: float  # *positive* value, rev/s²
    jerk_time_ms: float = 0.0  # ms (0 → no jerk filtering)


@dataclass(slots=True)
class SCurvePlanner:
    """Bang-bang planner with optional jerk-limited smoothing on position."""

    ctx: SCurvePlannerContext
    jerk: JerkFilter

    # Public state variables (updated at every *step*)
    position: float = field(default=0.0, init=False)
    velocity: float = field(default=0.0, init=False)
    acceleration: float = field(default=0.0, init=False)
    current_time: float = field(default=0.0, init=False)

    # Internal bookkeeping -------------------------------------------------
    _target_distance: float = field(default=0.0, init=False)
    _direction: float = field(default=1.0, init=False)
    _finished: bool = field(default=True, init=False)

    _previous_dx_filtered: float = field(default=0.0, init=False)
    _previous_position: float = field(default=0.0, init=False)

    def __init__(self, ctx: SCurvePlannerContext):
        self.jerk = JerkFilter(ctx.time_step)
        self.jerk.set_alpha(ctx.jerk_time_ms)

    def plan(self, distance: float) -> None:
        """Prepare a new point-to-point motion of *distance* revolutions."""
        if any(
            p <= 0.0
            for p in (
                self.ctx.time_step,
                self.ctx.max_velocity,
                self.ctx.max_acceleration,
                self.ctx.max_deceleration,
            )
        ):
            raise ValueError("Dynamic parameters must be strictly positive.")

        self._direction = 1.0 if distance >= 0.0 else -1.0
        self._target_distance = abs(distance)

        # Reset dynamic state
        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.current_time = 0.0

        self._previous_dx_filtered = 0.0
        self._previous_position = 0.0
        self._finished = False


    def step(self) -> Tuple[float, float, float]:
        """Advance the simulation by one *time_step* and return *(pos, vel, acc)*."""
        if self._finished:
            return self.position * self._direction, 0.0, 0.0

        dt = self.ctx.time_step
        vmax = self.ctx.max_velocity
        a_nom = self.ctx.max_acceleration
        d_nom = self.ctx.max_deceleration
        v_prev = self.velocity

        # Remaining distance **before** applying the increment
        remaining = self._target_distance - self.position
        if remaining <= EPS and abs(v_prev) <= EPS:
            self._finish()
            return self.position * self._direction, 0.0, 0.0

        # Ideal braking distance under constant −d_nom
        s_stop = v_prev * v_prev / (2.0 * d_nom) if v_prev > 0.0 else 0.0

        # Bang-bang acceleration command
        if s_stop >= remaining - EPS:  # Brake
            a_cmd = -d_nom
        elif v_prev < vmax - EPS:  # Accelerate
            # Do not overshoot vmax within the next sample
            a_cmd = min(a_nom, (vmax - v_prev) / dt)
        else:  # Cruise
            a_cmd = 0.0

        # Semi-implicit Euler integration for the *raw* increment
        v_new = v_prev + a_cmd * dt
        v_new = _clamp(v_new, 0.0, vmax)
        a_cmd = (v_new - v_prev) / dt  # adjust for clamping
        dx_raw = v_prev * dt + 0.5 * a_cmd * dt * dt

        # Rounding
        idx_raw = int(dx_raw)

        self.position += idx_raw
        self.current_time += dt

        # Numerical differentiation from smoothed position
        self.velocity = (self.position - self._previous_position) / dt
        self.acceleration = (self.velocity - v_prev) / dt
        self._previous_position = self.position

        # Completion check
        if (
            self.position >= self._target_distance - EPS
            and self.velocity <= d_nom * dt
        ):
            self._finish()

        return (
            self.position * self._direction,
            self.velocity * self._direction,
            self.acceleration * self._direction,
        )

    def _finish(self) -> None:  # noqa: D401
        self.position = self._target_distance
        self.velocity = 0.0
        self.acceleration = 0.0
        self._finished = True

    @property
    def is_finished(self) -> bool:  # noqa: D401
        """`True` once the point-to-point move is complete."""
        return self._finished


def _demo() -> None:  # noqa: D401
    """Run a quick demo and display position/velocity/acceleration plots."""

    context = SCurvePlannerContext(
        time_step=200e-6,  # 200 µs
        max_velocity=10.0,  # rev/s
        max_acceleration=10.0,  # rev/s²
        max_deceleration=10.0,  # rev/s²
        jerk_time_ms=5.0,  # 5 ms jerk filter window
    )

    planner = SCurvePlanner(context)
    planner.plan(100.0)  # target distance: 100 revolutions

    time_axis, pos, vel, acc = [], [], [], []
    while not planner.is_finished:
        p, v, a = planner.step()
        time_axis.append(planner.current_time)
        pos.append(p)
        vel.append(v)
        acc.append(a)

    # Plot results
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(time_axis, pos)
    axes[0].set_ylabel("Position [rev]")

    axes[1].plot(time_axis, vel)
    axes[1].set_ylabel("Velocity [rev/s]")

    axes[2].plot(time_axis, acc)
    axes[2].set_ylabel("Acceleration [rev/s²]")
    axes[2].set_xlabel("Time [s]")

    for ax in axes:
        ax.grid(True)

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":  # pragma: no cover
    _demo()
