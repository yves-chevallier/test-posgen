import numpy as np
import matplotlib.pyplot as plt

# Paramètres de filtrage (IIR 1er ordre – constante de temps 10 ms)
dt   = 1.0 / 5_000          # période d'échantionnage
tau  = 1e-3                # constante de temps
alpha = dt / tau

def generate_signal(segments, dtype=float):
    signal = []
    for (prev_val, (duration, value)) in zip([segments[0][1]] + [v for _, v in segments[:-1]], segments):
        ramp = (np.full(duration, value, dtype=float) if value == prev_val
                else np.linspace(prev_val, value, duration, dtype=dtype))
        signal.append(ramp)
    return np.concatenate(signal)

class JerkFilter:
    def __init__(self, dt: float):
        self.dt = dt
        self.alpha = 0.0
        self.reset()

    def reset(self):
        self.err = 0.0 # reliquat signé
        self.y_old = 0.0

    def set_alpha(self, tau = 10e-3):
        self.alpha = self.dt / tau if tau > 0.0 else 0.0

    def update(self, sample: int):
        if self.alpha == 0.0:
            return (sample, sample)

        x_eff = sample + self.err # restitution du résidu
        y = self.y_old + self.alpha * (x_eff - self.y_old)
        self.y_old = y
        iy = int(y)  # quantification entière
        self.err = y - iy # nouveau résidu
        return (iy, y)

jf = JerkFilter(dt)
jf.set_alpha(tau)

signal = generate_signal([(5, 0), (30, 100000), (10, 0), (30, 0)], dtype=int)

# Filtrage + compensation d'arrondierror–feedback ΔΣ ordre 1)
y_prev = signal[0]        # pré‑charge pour éviter le trou initial
err    = 0.0              # reliquat signé
y_float, y_int = [], []

for x in signal:
    iy, y = jf.update(x)
    # x_eff = x + err                       # restitution du résidu
    # y     = y_prev + alpha * (x_eff - y_prev)
    # y_prev = y
    # iy    = int(round(y))                 # quantification entière
    # err   = y - iy                        # nouveau résidu
    y_float.append(y)
    y_int.append(iy)


# Affichage
t = np.arange(len(signal))
plt.figure(figsize=(10, 4))
#plt.step(t, signal,  where="post", label="Entrée (excitation)", linewidth=1.2)
#plt.step(t, y_int,   where="post", label="Sortie quantifiée",   linewidth=1.2)
plt.stem(t, signal,  label="Entrée (excitation)", basefmt=" ")
plt.stem(t, y_int, "--",         label="Sortie flottante",     basefmt=" ")
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
