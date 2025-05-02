import numpy as np
import matplotlib.pyplot as plt

dt   = 1.0 / 1_000  # 1 kHz
tau  = 10e-3        # 10 ms  ->   α = dt/τ
alpha = dt / tau    # coefficient de l'IIR 1e, rdre (gain DC = 1)

data = np.array([0,0,0,1,2,3,4,5,6,5,4,3,2,2,2,2,2,2,2,2], dtype=float)

y_prev = data[0]    # amorçage (précharge)
ey = 0.0            # reliquat signé
ys_float   = []     # sortie haute précision
ys_integer = []     # sortie quantifiée

for x in data:
    x_eff = x + ey
    y = y_prev + alpha * (x_eff - y_prev)
    y_prev = y

    iy = int(round(y))  # entier le plus proche
    ey = y - iy         # reste signé  ∈  [-0.5, +0.5]

    ys_float.append(y)
    ys_integer.append(iy)

print("Somme entrée      :", data.sum())
print("Somme sortie int. :", sum(ys_integer))

t = np.arange(len(data))
plt.step(t, data,         label="Entrée",            where="post")
plt.step(t, ys_integer,   label="Sortie quantifiée", where="post")
plt.plot(t, ys_float,     label="Sortie flottante",  linestyle="--", alpha=0.6)
plt.title("IIR 1ᵉʳ ordre ± feedback d'erreur, τ = 10 ms")
plt.xlabel("Index échantillon")
plt.ylabel("Amplitude")
plt.legend()
plt.tight_layout()
plt.show()
