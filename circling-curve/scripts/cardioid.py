import numpy as np
import matplotlib.pyplot as plt

# Parâmetros
t = np.linspace(0, 2*np.pi, 40)  # Intervalo de t de 0 a 2pi
# x = 2 * np.cos(t) - np.cos(2*t)     # Equação x(t) do cardióide
# y = 2 * np.sin(t) - np.sin(2*t)     # Equação y(t) do cardióide
# x = 2 * np.cos(t) * (1 - np.cos(t))
# y = 2 * np.sin(t) * (1 - np.cos(t))

x = (1-2*np.cos(t)) * np.cos(t)
y = (1-2*np.cos(t)) * np.sin(t)
waypoints = np.column_stack([x,y])
print(waypoints)
# Visualização da trajetória
plt.plot(x, y)
plt.title('Trajetória do Robô - Cardióide')
plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')
plt.grid(True)
plt.show()