import numpy as np
import matplotlib.pyplot as plt

t = np.linspace(0, 2*np.pi, 100)
# x = 2 * np.cos(t) * (1 - np.cos(t))
# y = 2 * np.sin(t) * (1 - np.cos(t))

# limasson
# x = (1-2*np.cos(t)) * np.cos(t)
# y = (1-2*np.cos(t)) * np.sin(t)

x = 2 * np.cos(t)
y = 1 * np.sin(3*t)

waypoints = np.column_stack([x,y])
print(waypoints)
print(waypoints)

plt.plot(x, y)
plt.title('Curva de Lissajous')
plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')
plt.grid(True)
plt.show()