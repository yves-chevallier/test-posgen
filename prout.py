"""Bang‑bang trajectory generator with analytical deceleration criterion
===================================================================

This planner drives a single‑axis motion using three constant acceleration
levels:

* **+A_nom** – accelerate until either the nominal velocity is reached or the
  braking criterion is triggered.
* **0**      – hold the nominal velocity when there is still sufficient
  distance left to decelerate safely.
* **−D_nom** – decelerate as soon as the analytical braking distance exceeds
  the remaining distance to the target.

The braking distance is computed with the work–energy relationship
\(s_\text{stop} = v^2 / (2·D_\text{nom})\), guaranteeing an exact stop on the
command position, independent of the fixed integration step. Jerk is not part
of the kinematic integration; it is only computed (and optionally filtered) for
feed‑forward or monitoring purposes.

Public API
----------
* ``plan(distance: float)`` – initialise a new move.
* ``step() -> tuple[float, float, float, float]`` – advance the simulation by
  ``time_step`` seconds and return *(position, velocity, acceleration, jerk)*.
* ``is_finished`` – ``True`` when the motion is complete.

All physical units are coherent (revolution, rev/s, rev/s², rev/s³).
"""

import math
from dataclasses import dataclass
from typing import Tuple

import matplotlib.pyplot as plt

EPS = 1e-12  # numerical guard


def _clamp(x: float, low: float, high: float) -> float:
    """Return *x* limited to the interval [low, high]."""
    return max(low, min(high, x))


@dataclass
class SCurvePlanner:
    """Bang‑bang motion planner with an analytical deceleration criterion."""

    # User‑configurable parameters
    time_step: float                     # integration step [s]
    max_velocity: float = 0.0            # nominal velocity [rev/s]
    max_acceleration: float = 0.0        # accelerating slope [rev/s²]
    max_deceleration: float = 0.0        # braking slope (positive) [rev/s²]

    jerk_filter_alpha: float = 1.0       # 1 → no filtering, 0 → heavy smoothing

    # Public state variables (read‑only for the caller)
    position: float = 0.0                # [rev]
    velocity: float = 0.0                # [rev/s]
    acceleration: float = 0.0            # [rev/s²]
    jerk: float = 0.0                    # filtered jerk [rev/s³]
    current_time: float = 0.0            # [s]

    # Internal fields
    _target_distance: float = 0.0        # always non‑negative [rev]
    _direction: float = 1.0              # +1 or −1 depending on commanded sign
    _finished: bool = True

    _jerk_raw_prev: float = 0.0          # previous unfiltered jerk

    def plan(self, distance: float) -> None:
        """Initialise a new point‑to‑point move of *distance* revolutions."""
        if any(p <= 0 for p in (
            self.time_step,
            self.max_velocity,
            self.max_acceleration,
            self.max_deceleration,
        )):
            raise ValueError("All dynamic parameters must be strictly positive.")

        self._direction = 1.0 if distance >= 0 else -1.0
        self._target_distance = abs(distance)

        # Reset dynamic state
        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.jerk = 0.0
        self._jerk_raw_prev = 0.0
        self.current_time = 0.0
        self._finished = False

    def step(self) -> Tuple[float, float, float, float]:
        """Advance the simulation by ``time_step`` seconds."""
        if self._finished:
            return self.position * self._direction, 0.0, 0.0, 0.0

        dt = self.time_step

        # Remaining distance (always non‑negative)
        s_rem = self._target_distance - self.position
        if s_rem <= EPS and self.velocity <= EPS:
            self._finalise()
            return self.position * self._direction, 0.0, 0.0, 0.0

        v = self.velocity
        a_nom = self.max_acceleration
        d_nom = self.max_deceleration
        v_nom = self.max_velocity

        # Analytical braking distance with constant −D_nom
        s_stop = v * v / (2.0 * d_nom)

        # Acceleration command selection
        if s_stop >= s_rem - EPS:                 # braking phase
            a_cmd = -d_nom
        elif v < v_nom - EPS:                    # accelerating phase
            a_cmd = a_nom
            if v + a_cmd * dt > v_nom:           # avoid overshoot of V_nom
                a_cmd = (v_nom - v) / dt
        else:                                    # cruise phase
            a_cmd = 0.0

        # Kinematic integration
        v_new = v + a_cmd * dt
        v_new = _clamp(v_new, 0.0, v_nom)
        a_cmd = (v_new - v) / dt  # recompute if clamped

        self.position += v * dt + 0.5 * a_cmd * dt * dt
        prev_acc = self.acceleration
        self.velocity = v_new
        self.acceleration = a_cmd

        # Jerk calculation with optional low‑pass filter
        jerk_raw = (a_cmd - prev_acc) / dt
        alpha = self.jerk_filter_alpha
        self.jerk = alpha * jerk_raw + (1.0 - alpha) * self.jerk
        self._jerk_raw_prev = jerk_raw

        self.current_time += dt

        # Finalise when the target is reached and residual velocity is low
        if self.position >= self._target_distance - EPS and v_new <= d_nom * dt:
            self._finalise()

        return (
            self.position * self._direction,
            self.velocity * self._direction,
            self.acceleration * self._direction,
            self.jerk * self._direction,
        )

    def _finalise(self) -> None:
        """Clamp the final sample exactly on the command position."""
        self.position = self._target_distance
        self.velocity = 0.0
        self.acceleration = 0.0
        self.jerk = 0.0
        self._finished = True

    @property
    def is_finished(self) -> bool:  # noqa: D401 – property not a method
        """Return ``True`` when the move has completed."""
        return self._finished


if __name__ == "__main__":
    dt = 200e-6  # 200 µs integration step
    planner = SCurvePlanner(
        time_step=dt,
        max_velocity=10.0,
        max_acceleration=10.0,
        max_deceleration=10.0,
        jerk_filter_alpha=0.01,  # 20 % raw jerk, 80 % previous value
    )
    planner.plan(100.0)  # target: 100 revolutions

    t_axis, pos, vel, acc, jrk = [], [], [], [], []
    while not planner.is_finished:
        x, v, a, j = planner.step()
        t_axis.append(planner.current_time)
        pos.append(x)
        vel.append(v)
        acc.append(a)
        jrk.append(j)

    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    axes[0].plot(t_axis, pos)
    axes[0].set_ylabel("Position [rev]")

    axes[1].plot(t_axis, vel)
    axes[1].set_ylabel("Velocity [rev/s]")

    axes[2].plot(t_axis, acc)
    axes[2].set_ylabel("Acceleration [rev/s²]")

    axes[3].plot(t_axis, jrk)
    axes[3].set_ylabel("Jerk [rev/s³]")
    axes[3].set_xlabel("Time [s]")

    for ax in axes:
        ax.grid(True)

    fig.tight_layout()
    plt.show()
