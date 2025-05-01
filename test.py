import sys
import os
sys.path.insert(0, os.path.abspath("build"))

import positiongen
import matplotlib.pyplot as plt

gen = positiongen.PositionGenerator()
gen.init(50e-6 * 4)  # par exemple, 1 ms

gen.move(-30000, 400.0, 5000.0, 5000.0)

positions = []
pos = 0

while True:
    delta = gen.update()
    pos += delta
    positions.append(pos)
    if gen.target_reached():
        break

print(len(positions))
print("Final position:", pos)

plt.plot(positions)
plt.title("Position vs Time (steps)")
plt.xlabel("Time step")
plt.ylabel("Position")
plt.grid(True)
plt.show()
