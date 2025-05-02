from dataclasses import dataclass, field
from typing import Tuple
import matplotlib.pyplot as plt  # type: ignore
import numpy as np

EPS = 1e-12

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(x, hi))



@dataclass(slots=True)
class SCurvePlanner:
    time_step: float            # s
    max_velocity: float         # rev / s
    max_acceleration: float     # rev / s²
    max_deceleration: float     # rev / s²

    # états exposés
    position: float = field(default=0.0, init=False)
    velocity: float = field(default=0.0, init=False)
    acceleration: float = field(default=0.0, init=False)
    current_time: float = field(default=0.0, init=False)

    # internes
    _target: float = field(default=0.0, init=False)
    _dir: float = field(default=1.0, init=False)
    _finished: bool = field(default=True, init=False)

    _pos_int: int = field(default=0, init=False)     # position discrète (inc)
    _frac:    float = field(default=0.0, init=False) # éventuelle accumulation

    def plan(self, distance: int) -> None:
        self._dir = 1.0 if distance >= 0 else -1.0
        self._target = abs(distance)
        self.position = self.velocity = self.acceleration = self.current_time = 0.0
        self._pos_int = 0
        self._frac = 0.0
        self._finished = False

    def step(self) -> Tuple[float, float, float]:
        if self._finished:
            return self._dir * self.position, 0.0, 0.0

        # -------- paramètres locaux --------
        dt     = self.time_step
        vmax   = self.max_velocity
        a_max  = self.max_acceleration
        d_max  = self.max_deceleration

        v0        = self.velocity
        remaining = self._target - self.position

        # arrêt quasi‑statique
        if remaining <= 1 and abs(v0) <= EPS:
            self._finish()
            return self._dir * self.position, 0.0, 0.0

        # -------- 1.  critère de décélération anticipée --------
        s_stop = v0 * dt + v0 * v0 / (2.0 * d_max)
        if s_stop >= remaining - EPS:           # on doit freiner
            a_cmd = -d_max
        elif v0 < vmax - EPS:                   # on peut encore accélérer
            a_cmd = min(a_max, (vmax - v0) / dt)
        else:                                   # palier
            a_cmd = 0.0

        # -------- 2.  intégration cinématique --------
        v1    = clamp(v0 + a_cmd * dt, 0.0, vmax)
        a_cmd = clamp((v1 - v0) / dt, -d_max, a_max)
        dx    = int(v0 * dt + 0.5 * a_cmd * dt * dt)

        # -------- 3.  sûreté terminale --------
        if dx > remaining:
            dx = remaining
            v1   = max(0.0, 2.0 * dx / dt - v0)          # continuité énergétique
            a_cmd = clamp((v1 - v0) / dt, -d_max, a_max)

        # -------- 4.  mise à jour d’état --------
        self.position     += dx
        self.current_time += dt
        self.velocity      = v1
        self.acceleration  = a_cmd

        # 2. projection sur la grille d'incréments -------------------------
        p_int   = int(round(self._dir * self.position))   # position discrète visée
        dp      = p_int - self._pos_int                   # déplacement à effectuer
        self._pos_int = p_int                             # mémorise pour l'itération suivante

        # 3. test de fin (inchangé) ----------------------------------------
        if self.position >= self._target - EPS and self.velocity <= d_max * dt:
            self._finish()

        return dp, self._dir * self.velocity, self._dir * self.acceleration


    @property
    def is_finished(self) -> bool:
        return self._finished

    def _finish(self) -> None:
        self.position = self._target
        self.velocity = self.acceleration = 0.0
        self._finished = True


planner = SCurvePlanner(    time_step=200e-6,
    max_velocity=100000,
    max_acceleration=16000000,
    max_deceleration=16000000)
planner.plan(1000)

time_axis, pos, vel, acc = [], [], [], []

dx = 0.0
p_prev = 0.0
pp = 0.0
p = 0
count = 10000
while count > 0 and not planner.is_finished:
    dp, v, a = planner.step()
    p += dp
    time_axis.append(planner.current_time)
    dx = p - p_prev
    p_prev = p
    pp += dx

    pos.append(pp)
    vel.append(v)
    acc.append(a)

    if planner.is_finished:
        count -= 1

pos = np.array(pos)
vel = np.array(vel)
acc = np.array(acc)

# Plot results
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
axes[0].plot(time_axis, pos, marker="o", markerfacecolor='none', label="Position")
axes[0].set_ylabel("Position [rev]")

axes[1].plot(time_axis, vel, marker="o", markerfacecolor='none')
axes[1].set_ylabel("Velocity [rev/s]")

axes[2].plot(time_axis, acc, marker="o", markerfacecolor='none')
axes[2].set_ylabel("Acceleration [rev/s²]")
axes[2].set_xlabel("Time [s]")

for ax in axes:
    ax.grid(True)

fig.tight_layout()
plt.show()
