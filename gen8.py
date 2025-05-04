from scurve import SCurvePlanner, ContextScurvePlanner, Distretizer

import matplotlib.pyplot as plt

dt = 200e-6

planner = SCurvePlanner(dt=dt)

context = ContextScurvePlanner(
    velocityLimit=1000,
    acceleration=50000,
    deceleration=50000,
    jerk_time=0
)

planner.set_context(context)
planner.set_distance(10)

p = t = 0
time, pos, vel, acc = [], [], [], []
posi = []

planner.start()

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

        print(sample, x, self._err)
        return x

distretizer = Distretizer()

pz = 0
while not planner.finished():
    v, a = planner.step()

    # Integration
    p += v
    t += dt

    pz += distretizer.step(v)

    time.append(t)
    pos.append(p)
    posi.append(pz)
    vel.append(v / dt)
    acc.append(a / dt / dt)

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
