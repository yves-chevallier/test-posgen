from scurve import SCurvePlanner, ContextScurvePlanner, Distretizer

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
