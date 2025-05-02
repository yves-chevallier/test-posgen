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

        # 1. critère de freinage identique
        s_stop = v0 * dt + v0 * v0 / (2.0 * d_max)
        if s_stop >= remaining - EPS:
            a_cmd = -d_max
        elif v0 < vmax - EPS:
            a_cmd = min(a_max, (vmax - v0) / dt)
        else:
            a_cmd = 0.0

        # 2. vitesse finale réellement atteignable
        v1    = clamp(v0 + a_cmd * dt, 0.0, vmax)
        a_cmd = clamp((v1 - v0) / dt, -d_max, a_max)

        # 3. déplacement cohérent (trapèze)
        dx_f  = 0.5 * (v0 + v1) * dt

        # 4. sûreté terminale
        if dx_f > remaining:
            dx_f = remaining
            v1   = max(0.0, 2.0 * dx_f / dt - v0)
            a_cmd = clamp((v1 - v0) / dt, -d_max, a_max)

        # 5. mise à jour d’état (inchangé)
        self.position     += dx_f
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

class JerkFilter:
    def __init__(self, dt: float):
        self.dt = dt
        self.alpha = 0.0
        self.reset()

    def reset(self):
        self.err = 0.0 # reliquat signé
        self.y_old = 0.0

    def set_tau(self, tau = 10e-3):
        self.alpha = self.dt / tau if tau > 0.0 else 0.0

    def update(self, sample: int):
        if self.alpha == 0.0: return (sample, sample)

        x_eff = sample + self.err # restitution du résidu
        y = self.y_old + self.alpha * (x_eff - self.y_old)
        self.y_old = y
        iy = int(round(y))  # quantification entière
        self.err = y - iy # nouveau résidu
        return (iy, y)

dt = 200e-6

planner = SCurvePlanner(    time_step=dt,
    max_velocity=1000,
    max_acceleration=1600,
    max_deceleration=1600)
planner.plan(1000)

jf = JerkFilter(dt)
jf.set_tau(1e-3)

time_axis, pos, posj, vel, acc, dps = [], [], [], [], [], []

dx = 0.0
p_prev = 0.0
pp = 0.0
p = 0
pj = 0
count = 10000

while count > 0 and not planner.is_finished:
    dp, v, a = planner.step()
    p += dp
    pj += jf.update(dp)

    time_axis.append(planner.current_time)
    pos.append(p)
    posj.append(pj)
    dps.append(dp)
    vel.append(v)
    acc.append(a)

    if planner.is_finished:
        count -= 1

pos = np.array(pos)
posj = np.array(posj)
vel = np.array(vel)
acc = np.array(acc)

# Diff np.diff même taille que pos
v = np.diff(posj, prepend=pos[0]) / dt
a = np.diff(v, prepend=vel[0]) / dt

vv = np.diff(pos, prepend=pos[0]) / dt
aa = np.diff(vv, prepend=vel[0]) / dt

# Plot results
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
axes[0].plot(time_axis, pos,  markerfacecolor='none', label="Position")
axes[0].plot(time_axis, posj,  markerfacecolor='none', label="Position")
axes[0].set_ylabel("Position [rev]")

axes[1].plot(time_axis, vel,  markerfacecolor='none')
axes[1].plot(time_axis, v,  markerfacecolor='none')
axes[1].plot(time_axis, np.array(dps)/dt,  markerfacecolor='none', label="Position (inc)")
axes[1].set_ylabel("Velocity [rev/s]")

axes[2].plot(time_axis, acc,  markerfacecolor='none')
axes[2].plot(time_axis, a,  markerfacecolor='none')
axes[2].plot(time_axis, aa,  markerfacecolor='none')
axes[2].set_ylabel("Acceleration [rev/s²]")
axes[2].set_xlabel("Time [s]")

for ax in axes:
    ax.grid(True)

fig.tight_layout()
plt.show()
