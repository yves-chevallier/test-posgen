import numpy as np
import matplotlib.pyplot as plt

# Paramètres de filtrage (IIR 1er ordre – constante de temps 10 ms)
dt   = 1.0 / 5_000          # période d'échantionnage
tau  = 50e-3                # constante de temps
alpha = dt / tau

def generate_signal(segments):
    signal = []
    for (prev_val, (duration, value)) in zip([segments[0][1]] + [v for _, v in segments[:-1]], segments):
        ramp = (np.full(duration, value, dtype=float) if value == prev_val
                else np.linspace(prev_val, value, duration, dtype=float))
        signal.append(ramp)
    return np.concatenate(signal)

signal = generate_signal([(50, 0), (200, 100), (200, 0), (3000, 0)])

# Filtrage + compensation d'arrondierror–feedback ΔΣ ordre 1)
y_prev = signal[0]        # pré‑charge pour éviter le trou initial
err    = 0.0              # reliquat signé
y_float, y_int = [], []

for x in signal:
    x_eff = x + err                       # restitution du résidu
    y     = y_prev + alpha * (x_eff - y_prev)
    y_prev = y
    iy    = int(round(y))                 # quantification entière
    err   = y - iy                        # nouveau résidu
    y_float.append(y)
    y_int.append(iy)

# Affichage
t = np.arange(len(signal))
plt.figure(figsize=(10, 4))
plt.step(t, signal,  where="post", label="Entrée (excitation)", linewidth=1.2)
plt.step(t, y_int,   where="post", label="Sortie quantifiée",   linewidth=1.2)
plt.plot(t, y_float, "--",         label="Sortie flottante",    alpha=0.6)
plt.title("IIR 1ᵉʳ ordre + feedback d'erreur τ = 10 ms")
plt.xlabel("Indice échantillon")
plt.ylabel("Amplitude (unités entières)")
plt.legend()
plt.tight_layout()
plt.show()

# Vérification des sommes
s_in  = int(signal.sum())
s_out = sum(y_int)
print(f"Somme entrée  : {s_in}")
print(f"Somme sortie  : {s_out}")
print(f"Différence    : {s_in - s_out}")
