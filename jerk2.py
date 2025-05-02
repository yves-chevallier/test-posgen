import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

# ------------------------------------------------------------------
def magic_u32(divisor: int) -> Tuple[int, int]:
    l = (divisor - 1).bit_length()
    m = (1 << (32 + l)) // divisor
    if ((1 << (32 + l)) % divisor) != 0:
        m += 1
    return m, 32 + l

class StableIIRFilterExact:
    __slots__ = ("_state", "_mult", "_shift", "_leak")

    def __init__(self, leak: int, prime: int):
        self._leak = leak
        self._mult, self._shift = magic_u32(leak)
        self._state = prime * leak  # pre‑charge

    def _div_leak(self, x: int) -> int:
        return (x * self._mult) >> self._shift

    def update(self, x: int) -> int:
        q = self._div_leak(self._state)
        self._state += x - q
        return self._div_leak(self._state)

class JerkIIRFilterExact:
    __slots__ = ("_f1", "_f2")

    def __init__(self, leak: int, initial_dx: int):
        self._f1 = StableIIRFilterExact(leak, initial_dx)
        self._f2 = StableIIRFilterExact(leak, initial_dx)

    def update(self, dx: int) -> int:
        v1 = self._f1.update(dx)
        v2 = self._f2.update(v1)
        return v2

def compute_leak(jerk_time_ms: float, dt_sec: float) -> int:
    return max(1, round((jerk_time_ms / 1000.0) / dt_sec))

# ------------------------------------------------------------------
# Parameters
dt_sec = 0.001
jerk_time_ms = 10
leak = compute_leak(jerk_time_ms, dt_sec)

# Velocity profile
n_const, n_ramp = 200, 100
v_low, v_high = 20, 80
phase1 = np.full(n_const, v_low, dtype=int)
phase2 = np.linspace(v_low, v_high, n_ramp, dtype=int)
phase3 = np.full(n_const, v_high, dtype=int)
phase4 = np.linspace(v_high, v_low, n_ramp, dtype=int)
phase5 = np.full(n_const, v_low, dtype=int)
signal = np.concatenate([phase1, phase2, phase3, phase4, phase5])

# Filter
jerk_filter = JerkIIRFilterExact(leak, int(signal[0]))
filtered = np.empty_like(signal)
for i, x in enumerate(signal):
    filtered[i] = jerk_filter.update(int(x))

# ------------------------------------------------------------------
# Plot
plt.figure(figsize=(10, 4))
x_axis = np.arange(len(signal))
plt.step(x_axis, signal, label="Original velocity", linewidth=1.2, where="post")
plt.step(x_axis, filtered, label="Filtered velocity", linewidth=1.2, where="post")
plt.title(f"Velocity before/after jerk‑limiting IIR (τ={jerk_time_ms} ms, leak={leak})")
plt.xlabel("Sample index")
plt.ylabel("Velocity (integer units)")
plt.legend()
plt.tight_layout()
plt.show()
