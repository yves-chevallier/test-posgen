from dataclasses import dataclass, field
from math import sqrt

@dataclass(slots=True)
class ContextScurvePlanner:

    vmax: float  = field(default=0.0, init=False)
    acc: float  = field(default=0.0, init=False)
    dec: float  = field(default=0.0, init=False)
    jerk: float = field(default=0.0, init=False)

class SCurvePlanner:
    """ Generate smooth S-Curve movement profile """

    def __init__(self, dt: float):
        self.dt = dt
        self.sign = 1.0

        self.x = 0.0
        self.acc = 0
        self.dec = 0
        self.vmax = 0
        self.alpha = 0

        self.v = 0.0

        self._v_old = 0.0
        self._dx_old = 0.0
        self._sign = 1.0
        self._breaking = False
        self._finished = True
        self._jerk_done = False

    def set_acceleration(self, value):
        """ Set acceleration in pu/s² """
        self.acc = value * self.dt

    def set_deceleration(self, value):
        """ Set deceleration in pu/s² """
        self.dec = value * self.dt

    def set_velocity(self, value):
        """ Set max velocity in pu/s """
        self.vmax = value

    def set_jerk(self, tau: float):
        """ Set jerk in seconds """
        self.alpha = self.dt / tau if tau > 0.0 else 0.0

    def set_distance(self, value):
        """ Set distance to travel in pu """
        self._sign = 1.0 if value >= 0 else -1.0
        self.x = abs(value)

    def stop(self, deceleration=None):
        """ Stop the movement """
        self.dec = deceleration if deceleration is not None else self.dec
        self._breaking = True

    def start(self):
        """ Start the movement """
        if not self._finished:
            return False

        self.v = 0
        self._finished = False
        self._breaking = False
        self._jerk_done = False
        return True

    def finished(self) -> bool:
        """ Check if the movement is finished """
        return self._finished and self._jerk_done

    def step(self):
        """ Compute the next step at `dt` interval """
        if self._finished:
            return self.apply_jerk(0)

        self._v_old = self.v

        if self._breaking and self.x > self.dec:
            self.v -= self.dec
            if self.v <= 0:
                self.v = 0
                self._finished = True

        if self.x <= self.dec:
            self.v = self.x
            self.x = 0
            self._finished = True
        else:
            # How many steps N to finish the movement?
            # x = 1/2 v * N + 1/2 dec * N = 1/2 dec (N**2 + N)
            # V = N * dec
            # x = V ( V + dec ) / (2 * dec)
            # thus:
            # 2 * x * dec <= V * (V + dec)
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

        return self.apply_jerk(self.v)

    def apply_jerk(self, sample):

        if self.alpha > 0.0:
            dx = self._dx_old + self.alpha * (sample - self._dx_old)
            self._jerk_done = abs(dx - self._dx_old) < 1e-3
        else:
            self._jerk_done = True
            dx = sample

        ret = (dx, dx - self._dx_old)
        self._dx_old = dx
        return ret


class Distretizer:
    """ Convert a float to an int with a residual error """
    def __init__(self):
        self._err = 0.0

    def step(self, sample):
        """ Convert a float to an int with a residual error """
        x = int(sample)
        self._err += sample - x

        if abs(self._err) >= 1.0:
            ierr = int(self._err)
            x += ierr
            self._err -= ierr

        return x

import numpy as np
import matplotlib.pyplot as plt

dt = 200e-6

planner = SCurvePlanner(dt=dt)
planner.set_velocity(20)
planner.set_acceleration(1000)
planner.set_deceleration(1000)
planner.set_jerk(2e-3)

planner.set_distance(10000)

p = t = 0
time, pos, vel, acc = [], [], [], []
posi = []

planner.start()

distretizer = Distretizer()


while not planner.finished():
    v, a = planner.step()

    # Integration
    p += v
    t += dt

    time.append(t)
    pos.append(p)
    posi.append(distretizer.step(p))
    vel.append(v)
    acc.append(a)

fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
axes[0].plot(time, pos, label="Position", marker='.')
axes[0].plot(time, posi, label="Position", marker='.')
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
