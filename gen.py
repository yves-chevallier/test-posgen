from dataclasses import dataclass, field
from typing import Tuple
import matplotlib.pyplot as plt  # type: ignore
import numpy as np

EPS = 1e-12
def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(x, hi))


class JerkFilter:
    """Filtre passe-bas du pas de position (s-courbe de 3ᵉ ordre)."""
    def __init__(self, dt: float, tau: float) -> None:
        self.dt, self.alpha = dt, dt / tau if tau > 0 else 0.0
        self.reset()
    def reset(self) -> None:
        self._err = self._y_old = 0.0
    def update(self, dx: float) -> float:
        if self.alpha == 0.0:
            return dx
        x_eff = dx + self._err
        y = self._y_old + self.alpha * (x_eff - self._y_old)
        self._y_old, self._err = y, x_eff - y
        return y


@dataclass(slots=True, frozen=True)
class SCurvePlannerContext:
    time_step: float            # s
    max_velocity: float         # rev / s
    max_acceleration: float     # rev / s²
    max_deceleration: float     # rev / s²
    jerk_time_ms: float = 0.0   # ms


@dataclass(slots=True)
class SCurvePlanner:
    ctx: SCurvePlannerContext
    _jerk: JerkFilter = field(init=False)

    # états exposés
    position: float = field(default=0.0, init=False)
    velocity: float = field(default=0.0, init=False)
    acceleration: float = field(default=0.0, init=False)
    current_time: float = field(default=0.0, init=False)

    # internes
    _target: float = field(default=0.0, init=False)
    _dir: float = field(default=1.0, init=False)
    _finished: bool = field(default=True, init=False)

    def __post_init__(self) -> None:
        tau_s = self.ctx.jerk_time_ms * 1e-3
        self._jerk = JerkFilter(self.ctx.time_step, tau_s)

    def plan(self, distance: float) -> None:
        self._dir = 1.0 if distance >= 0.0 else -1.0
        self._target = abs(distance)
        self.position = self.velocity = self.acceleration = self.current_time = 0.0
        self._jerk.reset()
        self._finished = False

    def step(self) -> Tuple[float, float, float]:
        if self._finished:
            return self._dir * self.position, 0.0, 0.0

        dt, vmax = self.ctx.time_step, self.ctx.max_velocity
        a_max, d_max = self.ctx.max_acceleration, self.ctx.max_deceleration
        v_prev = self.velocity
        remaining = self._target - self.position

        # arrêt si distance et vitesse quasi nulles
        if remaining <= EPS and abs(v_prev) <= EPS:
            self._finish()
            return self._dir * self.position, 0.0, 0.0

        s_stop = v_prev * v_prev / (2.0 * d_max) if v_prev > 0.0 else 0.0
        if s_stop >= remaining - EPS:          # freinage
            a_cmd = -d_max
        elif v_prev < vmax - EPS:              # accélération
            a_cmd = min(a_max, (vmax - v_prev) / dt)
        else:                                  # palier
            a_cmd = 0.0

        v_new = _clamp(v_prev + a_cmd * dt, 0.0, vmax)
        a_cmd = (v_new - v_prev) / dt
        dx = v_prev * dt + 0.5 * a_cmd * dt * dt

        # intégration
        self.position   += dx
        self.current_time += dt
        self.velocity     = v_new
        self.acceleration = a_cmd

        if self.position >= self._target - EPS and self.velocity <= d_max * dt:
            self._finish()

        return (
            self._dir * self.position,
            self._dir * self.velocity,
            self._dir * self.acceleration,
        )

    @property
    def is_finished(self) -> bool:
        return self._finished

    def _finish(self) -> None:
        self.position = self._target
        self.velocity = self.acceleration = 0.0
        self._finished = True



ctx = SCurvePlannerContext(
    time_step=200e-6,
    max_velocity=0.8,
    max_acceleration=200,
    max_deceleration=200,
    jerk_time_ms=50.0,
)
planner = SCurvePlanner(ctx)
planner.plan(0.01)

jerk = JerkFilter(ctx.time_step, ctx.jerk_time_ms * 1e-3)

time_axis, pos, posj, vel, acc = [], [], [], [], []

dx = 0.0
p_prev = 0.0
pp = 0.0
ppj = 0.0

count = 10000
while count > 0 and not planner.is_finished:
    p, v, a = planner.step()
    time_axis.append(planner.current_time)
    dx = p - p_prev
    p_prev = p
    pp += dx
    ppj += jerk.update(0)

    pos.append(pp)
    posj.append(ppj)
    vel.append(v)
    acc.append(a)

    if planner.is_finished:
        count -= 1

pos = np.array(pos)
vel = np.array(vel)
acc = np.array(acc)

vel_jerk = np.zeros_like(vel)
vel_jerk[1:] = np.diff(posj) / ctx.time_step

# Plot results
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
axes[0].plot(time_axis, pos, marker="o", markerfacecolor='none', label="Position")
axes[0].plot(time_axis, posj, label="Filtered position", linestyle="--")
axes[0].set_ylabel("Position [rev]")

axes[1].plot(time_axis, vel, marker="o", markerfacecolor='none')
axes[1].set_ylabel("Velocity [rev/s]")
axes[1].plot(time_axis, vel_jerk, label="Position diff", linestyle="--")

axes[2].plot(time_axis, acc, marker="o", markerfacecolor='none')
axes[2].set_ylabel("Acceleration [rev/s²]")
axes[2].set_xlabel("Time [s]")

for ax in axes:
    ax.grid(True)

fig.tight_layout()
plt.show()

print(
    f"Terminé : {planner.position:.2f} rev en {planner.current_time:.4f} s "
    f"v_max ≈ {ctx.max_velocity} rev/s"
)
