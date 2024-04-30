import numpy as np
import matplotlib.pyplot as plt

t = np.linspace(0, 2*np.pi, 40)
x = 2 * np.cos(t) * (1 - np.cos(t))
y = 2 * np.sin(t) * (1 - np.cos(t))

# limasson
# x = (1-2*np.cos(t)) * np.cos(t)
# y = (1-2*np.cos(t)) * np.sin(t)
waypoints = np.column_stack([x,y])
print(waypoints)

plt.plot(x, y)
plt.title('Trajetória do Robô - Cardióide')
plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')
plt.grid(True)
plt.show()