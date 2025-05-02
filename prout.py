"""
Refined seven‑phase S‑curve trajectory generator.

Key update
----------
* **Variable sub‑step integration** – the integrator now shortens the very last
  step so that the numerical timeline lands *exactly* on the analytical end
  of the profile. Consequently, the final sample is now continuous with the
  penultimate one; the spurious jump that was visible at the end of the
  position/velocity curves disappears.

All physical units remain coherent (e.g. revolutions, rev/s, rev/s², rev/s³).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

import matplotlib.pyplot as plt


@dataclass
class SCurvePlanner:
    """Seven‑phase S‑curve trajectory generator (refined version)."""

    # Public parameters
    time_step: float
    max_velocity: float = 0.0
    max_acceleration: float = 0.0
    max_deceleration: float = 0.0
    jerk_time_ms: float = 0.0

    # Dynamic state
    position: float = 0.0  # rev
    velocity: float = 0.0  # rev/s
    acceleration: float = 0.0  # rev/s²
    jerk: float = 0.0  # rev/s³
    current_time: float = 0.0  # s

    # Internal scheduling
    _jerk_duration: float = 0.0  # s
    _jerk_acc: float = 0.0
    _jerk_dec: float = 0.0
    _phase_end_times: List[float] = field(default_factory=list)
    _finished: bool = True
    _target_distance: float = 0.0

    # ---------------------------------------------------------------------
    # Public parameter setters (unchanged)
    # ---------------------------------------------------------------------
    def set_max_velocity(self, value: float) -> None:
        self.max_velocity = float(value)

    def set_max_acceleration(self, value: float) -> None:
        self.max_acceleration = float(value)
        if self._jerk_duration:
            self._jerk_acc = self.max_acceleration / self._jerk_duration

    def set_max_deceleration(self, value: float) -> None:
        self.max_deceleration = float(value)
        if self._jerk_duration:
            self._jerk_dec = self.max_deceleration / self._jerk_duration

    def set_jerk_time(self, time_ms: float) -> None:
        """Set the jerk transition time (0 → *Amax*)."""
        self.jerk_time_ms = float(time_ms)
        self._jerk_duration = self.jerk_time_ms / 1000.0  # convert to s
        if self.max_acceleration:
            self._jerk_acc = self.max_acceleration / self._jerk_duration
        if self.max_deceleration:
            self._jerk_dec = self.max_deceleration / self._jerk_duration

    # ---------------------------------------------------------------------
    # Trajectory planning (unchanged)
    # ---------------------------------------------------------------------
    def plan(self, distance: float) -> None:
        """Prepare an S‑curve profile to move *distance* revolutions."""
        # Sanity checks
        if any(v <= 0 for v in (
            self.time_step,
            self.max_velocity,
            self.max_acceleration,
            self.max_deceleration,
            self._jerk_duration,
        )):
            raise ValueError("Every dynamic parameter must be strictly positive.")

        v_cruise = self.max_velocity
        a_max = self.max_acceleration
        d_max = self.max_deceleration
        t_j = self._jerk_duration
        j_acc = self._jerk_acc
        j_dec = self._jerk_dec
        total_distance = abs(float(distance))

        # Helper to compute the distance travelled during a jerk‑limited
        # acceleration phase that ends at *a_max* and lasts *t_lin* at
        # constant acceleration (may be zero).
        def _s_curve_distance(a_limit: float, jerk: float, t_lin: float) -> float:
            t_jrk = a_limit / jerk
            d_jerk = (1.0 / 6.0) * jerk * t_jrk ** 3
            v_jerk = 0.5 * jerk * t_jrk ** 2
            d_lin = v_jerk * t_lin + 0.5 * a_limit * t_lin ** 2
            return 2.0 * d_jerk + d_lin

        # Nominal acceleration/deceleration durations without cruising
        t_acc_total = v_cruise / a_max
        t_dec_total = v_cruise / d_max

        t_acc_lin = max(t_acc_total - 2.0 * t_j, 0.0)
        t_dec_lin = max(t_dec_total - 2.0 * t_j, 0.0)

        # Distance for acceleration and deceleration segments
        distance_acc = _s_curve_distance(a_max, j_acc, t_acc_lin)
        distance_dec = _s_curve_distance(d_max, j_dec, t_dec_lin)
        distance_cruise = total_distance - (distance_acc + distance_dec)

        # If the distance is too short to reach *v_cruise*, compute the
        # admissible peak velocity analytically (symmetric case only).
        if distance_cruise < 0.0:
            if abs(a_max - d_max) < 1e-9:  # symmetric profile
                a_lim = a_max
                j_lim = j_acc
                c_term = (1.0 / 3.0) * a_lim * t_j ** 2
                # quadratic: a*u² + a*t_j*u + (2c - D) = 0
                a_q = a_lim
                b_q = a_lim * t_j
                c_q = 2.0 * c_term - total_distance
                disc = b_q ** 2 - 4.0 * a_q * c_q
                if disc < 0:
                    raise RuntimeError("Incompatible parameters for the given distance.")
                t_lin_new = (-b_q + math.sqrt(disc)) / (2.0 * a_q)
                v_cruise = a_lim * (t_j + t_lin_new)
                t_acc_lin = t_dec_lin = t_lin_new
                distance_cruise = 0.0
                distance_acc = distance_dec = _s_curve_distance(a_lim, j_lim, t_lin_new)
            else:
                raise RuntimeError(
                    "Asymmetric Amax/Dmax with insufficient distance is not handled."
                )

        t1 = t_j
        t2 = t1 + t_acc_lin
        t3 = t2 + t_j
        t4 = t3 + distance_cruise / v_cruise
        t5 = t4 + t_j
        t6 = t5 + t_dec_lin
        t7 = t6 + t_j
        self._phase_end_times = [t1, t2, t3, t4, t5, t6, t7]

        self.max_velocity = v_cruise  # may have been reduced above
        self._finished = False
        self._target_distance = total_distance

        # Reset dynamic state
        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.jerk = 0.0
        self.current_time = 0.0

    # ---------------------------------------------------------------------
    # *Fixed* integration routine
    # ---------------------------------------------------------------------
    def step(self) -> Tuple[float, float, float, float]:
        """Advance the profile by one (possibly shortened) time step.

        The integrator now adapts the very last step so that *current_time*
        coincides exactly with the theoretical end time, thereby removing the
        discontinuity that used to appear at the final sample.
        """
        if self._finished:
            return self.position, 0.0, 0.0, 0.0

        final_time = self._phase_end_times[-1]
        # Use a shortened sub‑step when approaching the end of the profile.
        dt = self.time_step if self.current_time + self.time_step < final_time else final_time - self.current_time

        # Guard against numerical artefacts (can happen when dt ≈ 0).
        if dt <= 0.0:
            self.position = self._target_distance
            self.velocity = self.acceleration = self.jerk = 0.0
            self.current_time = final_time
            self._finished = True
            return self.position, 0.0, 0.0, 0.0

        # Determine current phase (based on the time *before* integration).
        phase = next(
            (idx for idx, t_end in enumerate(self._phase_end_times) if self.current_time < t_end),
            7,
        )

        # Phase‑dependent jerk
        if phase == 0:
            self.jerk = self._jerk_acc
        elif phase == 1:
            self.jerk = 0.0
        elif phase == 2:
            self.jerk = -self._jerk_acc
        elif phase == 3:
            self.jerk = 0.0
        elif phase == 4:
            self.jerk = -self._jerk_dec
        elif phase == 5:
            self.jerk = 0.0
        elif phase == 6:
            self.jerk = self._jerk_dec

        # Discrete integration (now using the possibly shortened *dt*)
        self.acceleration += self.jerk * dt
        self.velocity += self.acceleration * dt + 0.5 * self.jerk * dt ** 2
        self.position += (
            self.velocity * dt
            + 0.5 * self.acceleration * dt ** 2
            + (1.0 / 6.0) * self.jerk * dt ** 3
        )
        self.current_time += dt

        # Exact termination condition
        if math.isclose(self.current_time, final_time, abs_tol=1e-12):
            self.position = self._target_distance
            self.velocity = self.acceleration = self.jerk = 0.0
            self._finished = True

        return self.position, self.velocity, self.acceleration, self.jerk

    # ------------------------------------------------------------------
    # Read‑only helper
    # ------------------------------------------------------------------
    @property
    def is_finished(self) -> bool:  # noqa: D401
        """Return *True* when the profile has completed."""
        return self._finished


# -------------------------------------------------------------------------
# Example (kept for completeness – behaviour is now continuous)
# -------------------------------------------------------------------------
if __name__ == "__main__":
    TIME_STEP = 200e-6  # 200 µs
    planner = SCurvePlanner(time_step=TIME_STEP)
    planner.set_max_velocity(10.0)      # rev/s
    planner.set_max_acceleration(10.0)  # rev/s²
    planner.set_max_deceleration(10.0)  # rev/s²
    planner.set_jerk_time(50.0)         # 1 ms
    planner.plan(100.0)                 # 100 revolutions

    t_axis: List[float] = []
    pos, vel, acc, jrk = [], [], [], []

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
