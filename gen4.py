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
    dt: float

    vmax: float  = field(default=0.0, init=False)
    acc: float  = field(default=0.0, init=False)
    dec: float  = field(default=0.0, init=False)

    # états exposés
    x: float = field(default=0.0, init=False)
    v: float = field(default=0.0, init=False)
    a: float = field(default=0.0, init=False)

    _sign: float = field(default=1.0, init=False)
    _finished: bool = field(default=True, init=False)
    _v_old: float = field(default=0.0, init=False)

    @property
    def is_finished(self) -> bool:
        return self._finished

    def plan(self, distance: int, vmax, acc: float, dec: float) -> None:
        self._sign = 1.0 if distance >= 0 else -1.0
        self.x = abs(distance)
        self.v = 0
        self._finished = False

        self.vmax = vmax
        self.acc = acc * dt
        self.dec = dec * dt

    def step(self) -> Tuple[int, float, float]:
        if self._finished:
            return 0.0, 0.0

        self._v_old = self.v

        if self.x <= self.dec - EPS:
            self.v = self.x
            self.x = 0
            self._finished = True
        else:
            left = 2 * self.dec * self.x
            right = self.v * (self.v + self.dec)

            if left <= right:
                self.v = (sqrt(self.dec ** 2 + 4 * left) - self.dec) / 2
            else:
                if self.v + self.acc > self.vmax:
                    self.v = self.v - self.dec if self.v > self.vmax else self.vmax
                else:
                    self.v += self.acc

            self.x -= self.v
            if self._v_old < self.v:
                left = 2 * self.dec * self.x
                right = self.v * (self.v + self.dec)
                if left <= right:
                    self.v = self._v_old
                    self.x += self.acc

        acc = (self.v - self._v_old)
        return self.v * self._sign, acc * self._sign


dt = 200e-6

planner = SCurvePlanner(dt=dt)
planner.plan(100, 2, 1000, 1000)

p = t = 0
time, pos, vel, acc = [], [], [], []
while not planner.is_finished:
    dx, a = planner.step()
    p += dx
    t += dt

    time.append(t)
    pos.append(p)
    vel.append(dx)
    acc.append(a)

fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
axes[0].stem(time, pos, label="Position")
axes[0].set_ylabel("Position [rev]")

axes[1].stem(time, vel)
axes[1].set_ylabel("Velocity [rev/s]")

axes[2].stem(time, acc)
axes[2].set_ylabel("Acceleration [rev/s²]")
axes[2].set_xlabel("Time [s]")

for ax in axes:
    ax.grid(True)

fig.tight_layout()
plt.show()
