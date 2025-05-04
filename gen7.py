from dataclasses import dataclass, field
from math import sqrt

@dataclass(slots=True)
class ContextScurvePlanner:
    velocityLimit: float
    acceleration: float
    deceleration: float
    jerk_time: float

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

    def set_context(self, ctx: ContextScurvePlanner):
        """ Set the context for the planner """
        self.ctx = ctx

    def apply_context(self):
        """ Apply context to the planner """
        self.acc = self.ctx.acceleration * self.dt
        self.dec = self.ctx.deceleration * self.dt
        self.vmax = self.ctx.velocityLimit
        self.alpha = self.dt / self.ctx.jerk_time if self.ctx.jerk_time > 0.0 else 0.0

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

        self.apply_context()
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
        """ Jerk is a simple first order low-pass filter.
        Do we really need more than that?
        """
        if self.alpha > 0.0:
            dx = self._dx_old + self.alpha * (sample - self._dx_old)
            self._jerk_done = abs(dx - self._dx_old) < 1e-6
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
        x = round(sample)
        self._err += sample - x

        if abs(self._err) >= 1.0:
            ierr = round(self._err)
            x += ierr
            self._err -= ierr

        return x

import matplotlib.pyplot as plt

dt = 200e-6

planner = SCurvePlanner(dt=dt)

context = ContextScurvePlanner(
    velocityLimit=1,
    acceleration=100,
    deceleration=100,
    jerk_time=1e-3
)

planner.set_context(context)
planner.set_distance(10)

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
kwargs = { 'marker': '.' } if len(time) < 500 else {}

if (len(time) > 100):
    axes[0].plot(time, posi, label="Position", **kwargs)
    axes[1].plot(time, vel, **kwargs)
    axes[2].plot(time, acc, **kwargs)
else:
    axes[0].stem(time, posi, label="Position Entière")
    axes[1].stem(time, vel)
    axes[2].stem(time, acc)

axes[0].plot(time, pos, label="Position")

axes[0].set_ylabel("Position [pu]")
axes[1].set_ylabel("Velocity [pu/s]")
axes[2].set_ylabel("Acceleration [pu/s²]")
axes[2].set_xlabel("Time [s]")

for ax in axes:
    ax.grid(True, which='major', linestyle='-')
    ax.minorticks_on()
    ax.grid(True, which='minor', linestyle=':', linewidth=0.5, alpha=0.7)

fig.tight_layout()
plt.show()
