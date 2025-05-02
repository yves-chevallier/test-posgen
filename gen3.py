from dataclasses import dataclass, field
from typing import Tuple
import matplotlib.pyplot as plt  # type: ignore
import numpy as np
from math import sqrt
EPS = 1e-12

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(x, hi))

@dataclass(slots=True)
class SCurvePlanner:
    dt: float            # s
    vmax: float         # rev / s
    acc: float     # rev / s²
    dec: float     # rev / s²

    # états exposés
    position: float = field(default=0.0, init=False)
    velocity: float = field(default=0.0, init=False)
    acceleration: float = field(default=0.0, init=False)
    current_time: float = field(default=0.0, init=False)

    _target: float = field(default=0.0, init=False)
    _sign: float = field(default=1.0, init=False)
    _finished: bool = field(default=True, init=False)
    _v_old: float = field(default=0.0, init=False)

    def plan(self, distance: int) -> None:
        self._sign = 1.0 if distance >= 0 else -1.0
        self.x = abs(distance)
        self.v = self.a = 0
        self._finished = False

    def step(self) -> Tuple[int, float, float]:
        if self._finished:
            return 0, 0.0, 0.0

        if self.x <= self.dec:
            self.v = self.x
            self.x = 0
            return 0, self._sign * v, 0

        left = 2 * self.dec * self.x
        right = self.v * (self.v + self.dec)

        if left <= right:
            # Start decelerating phase
            #v = v - dec
            self.v = (sqrt(self.dec ** 2 + 4 * left) - self.dec) / 2
        else:
            if v + acc > self.vmax:
                self.v = self.v - self.dec if self.v > self.vmax else self.vmax
            else:
                self.v += self.acc

        self.x -= self.v
        if self._v_old < self.v:
            left = 2 * self.dec * self.x
            right = self.v * (self.v + self.dec)
            if left <= right:
                self.v = self._v_old
                self.x += acc


        return self.x * self._sign, self.v * self._sign, 0

    @property
    def is_finished(self) -> bool:
        return self._finished

    def _finish(self) -> None:
        self.position = self._target
        self.velocity = self.acceleration = 0.0
        self._finished = True
        self._braking = False

dt = 200e-6

planner = SCurvePlanner(
    dt=dt,
    vmax=1000,
    acc=1600,
    dec=1600)

planner.plan(1000)

time_axis, pos, vel, acc = [], [], [], []

p = 0

movement_changed = False
t = 0
while not planner.is_finished:
    dp, v, a = planner.step()
    p += dp
    t += dt

    if p > 500 and not movement_changed:
        #planner.plan(800, v)
        planner.set_velocity(500)
        movement_changed = True

    time_axis.append(t)
    pos.append(p)
    vel.append(v)
    acc.append(a)

# Tester les critères
# 1. La vitesse ne doit jamais dépasser la vitesse limite.
# 2. L'accélération ne doit jamais dépasser l'accélération maximale.
# 3. Aucun saut dans l'accélération ne doit survenir durant la phase de décélération.
# 4. Le sens de déplacement ne dois jamais changer durant un mouvement.
# 5. La vitesse doit être nulle à la fin de la trajectoire.
# 6. Le mouvement doit pouvoir être changé à tout moment.
# 7. La vitesse peut être modifiée à tout moment, le système doit s'adapter.
# 8. La cible de position doit pouvoir être modifée à tout moment à condition que le les paramètres actuels le permettent.
# 9. L'accélération et la décélération doivent pouvoir être modifiées à tout moment à condition que le les paramètres actuels le permettent.

# Plot results
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
axes[0].plot(time_axis, pos,  markerfacecolor='none', label="Position")
axes[0].set_ylabel("Position [rev]")

axes[1].plot(time_axis, vel,  markerfacecolor='none')
axes[1].set_ylabel("Velocity [rev/s]")

axes[2].plot(time_axis, acc,  markerfacecolor='none')
axes[2].set_ylabel("Acceleration [rev/s²]")
axes[2].set_xlabel("Time [s]")

for ax in axes:
    ax.grid(True)

fig.tight_layout()
plt.show()

def check_profile(distance):
    planner.plan(distance)
    dp_hist, v_hist, a_hist = [], [], []
    while not planner.is_finished:
        dp, v, a = planner.step()
        dp_hist.append(dp); v_hist.append(v); a_hist.append(a)

    dp_hist, v_hist, a_hist = map(np.array, (dp_hist, v_hist, a_hist))

    assert np.abs(v_hist).max()      <= planner.vmax + 1e-9
    assert np.abs(a_hist).max()      <= max(planner.acc,
                                            planner.dec) + 1e-9

    # aucun retour d'accélération positive une fois le freinage engagé
    neg_idx = np.where(a_hist < -EPS)[0]
    if neg_idx.size:
        assert (a_hist[neg_idx[0]:] <= EPS).all()

    # signe constant
    if distance >= 0:
        assert (dp_hist >= 0).all()
    else:
        assert (dp_hist <= 0).all()

    assert v_hist[-1] == 0.0                       # vitesse finale
    assert dp_hist.sum() == abs(distance)          # position exacte

check_profile(1000)