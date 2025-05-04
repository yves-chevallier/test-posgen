import os
import sys
sys.path.insert(0, os.path.abspath("build"))

from scurvecpp import SCurvePlanner, ContextScurvePlanner, Distretizer
import matplotlib.pyplot as plt

# Paramètre de simulation
dt = 200e-6  # 200 µs

# Création du planificateur C++
planner = SCurvePlanner(dt)

# Contexte de mouvement (identique à la version Python)
context = ContextScurvePlanner()

context.velocityLimit=1.0
context.acceleration=100.0
context.deceleration=100.0
context.jerk_time=1e-3

planner.set_context(context)
planner.set_distance(10.0)
planner.start()

# Intégration temporelle et préparation des courbes
p = t = 0.0
time = []
pos = []
vel = []
acc = []
posi = []

distretizer = Distretizer()

while not planner.is_finished():
    v, a, done = planner.step()  # C++ binding retourne (v, a, is_finished)

    # Intégration simple de la position
    p += v
    t += dt

    time.append(t)
    pos.append(p)
    posi.append(distretizer.step(p))
    vel.append(v)
    acc.append(a)

# Affichage graphique
fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
kwargs = {'marker': '.'} if len(time) < 500 else {}

if len(time) > 100:
    axes[0].plot(time, posi, label="Position Entière", **kwargs)
    axes[1].plot(time, vel, **kwargs)
    axes[2].plot(time, acc, **kwargs)
else:
    axes[0].stem(time, posi, label="Position Entière")
    axes[1].stem(time, vel)
    axes[2].stem(time, acc)

axes[0].plot(time, pos, label="Position Continue")

axes[0].set_ylabel("Position [pu]")
axes[1].set_ylabel("Velocity [pu/s]")
axes[2].set_ylabel("Acceleration [pu/s²]")
axes[2].set_xlabel("Time [s]")

for ax in axes:
    ax.grid(True, which='major', linestyle='-')
    ax.minorticks_on()
    ax.grid(True, which='minor', linestyle=':', linewidth=0.5, alpha=0.7)

axes[0].legend()
fig.tight_layout()
plt.show()
