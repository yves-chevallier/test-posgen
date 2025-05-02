from dataclasses import dataclass, field
from typing import Tuple
import matplotlib.pyplot as plt  # type: ignore
import numpy as np
from math import sqrt


@dataclass(slots=True)
class SCurvePlanner:
    dt: float

    vmax: float  = field(default=0.0, init=False)
    acc: float  = field(default=0.0, init=False)
    dec: float  = field(default=0.0, init=False)
    jerk: float = field(default=0.0, init=False)

    # états exposés
    x: float = field(default=0.0, init=False)
    v: float = field(default=0.0, init=False)
    a: float = field(default=0.0, init=False)

    _sign: float = field(default=1.0, init=False)
    _finished: bool = field(default=True, init=False)
    _v_old: float = field(default=0.0, init=False)
    _err: float = field(default=0.0, init=False)
    dx_old: float = field(default=0.0, init=False)

    @property
    def is_finished(self) -> bool:
        return self._finished

    def plan(self, distance: int, vmax, acc: float, dec: float, jerk: float) -> None:
        self._sign = 1.0 if distance >= 0 else -1.0
        self.x = abs(distance)
        self.v = 0
        self._finished = False

        self.vmax = vmax
        self.acc = acc * dt
        self.dec = dec * dt

        self.jerk = self.dt / jerk

    def step(self) -> Tuple[int, float, float]:
        if self._finished:
            return 0, 0, 0

        self._v_old = self.v

        if self.x <= self.dec:
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

        # Apply jerk filter
        return self.apply_jerk(self.v)

    def apply_jerk(self, sample):

        if self.jerk > 0.0:
            dx = self.dx_old + self.jerk * (sample - self.dx_old)
        else:
            dx = sample

        a = dx - self.dx_old
        self.dx_old = dx

        # Discretisation
        iy = int(dx)
        self._err += dx - iy

        if abs(self._err) >= 1.0:
            iy += int(self._err)
            self._err -= int(self._err)

        return iy, dx, a

dt = 200e-6

planner = SCurvePlanner(dt=dt)
planner.plan(10000, 20, 1000, 1000, 10e-3)

p = t = 0
pj = 0
time, pos, vel, acc = [], [], [], []
posj, velj, accj = [], [], []

err = 0

count = 13
while count > 0:
    dx, v, a = planner.step()

    # Integration
    p += dx
    t += dt

    time.append(t)
    pos.append(p)
    vel.append(v)
    acc.append(a)

    if planner.is_finished:
        count -= 1

accj = np.diff(velj, prepend=0.0)

fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
axes[0].plot(time, pos, label="Position", marker='.')
axes[0].set_ylabel("Position [rev]")

axes[1].plot(time, vel, marker='.')
axes[1].set_ylabel("Velocity [rev/s]")

axes[2].plot(time, acc, marker='.')

axes[2].set_ylabel("Acceleration [rev/s²]")
axes[2].set_xlabel("Time [s]")

# axes[0].stem(time, pos, label="Position")
# axes[0].stem(time, posj, label="Position2", linefmt='g-', markerfmt='go', basefmt='k-' )
# axes[0].set_ylabel("Position [rev]")

# axes[1].stem(time, vel)
# axes[1].stem(time, velj, label="Position2", linefmt='g-', markerfmt='go', basefmt='k-' )
# axes[1].set_ylabel("Velocity [rev/s]")

# axes[2].stem(time, acc)
# axes[2].stem(time, accj, label="Position2", linefmt='g-', markerfmt='go', basefmt='k-' )
# axes[2].set_ylabel("Acceleration [rev/s²]")
# axes[2].set_xlabel("Time [s]")

for ax in axes:
    ax.grid(True)

fig.tight_layout()
plt.show()
