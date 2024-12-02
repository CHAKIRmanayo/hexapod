import numpy as np
import matplotlib.pyplot as plt

def foot_trajectory(t, L, h):
    if t < 0.5:  # Переносная фаза
        x = L * t
        z = 4 * h * t * (0.5 - t)
    else:  # Опорная фаза
        x = L * t
        z = 0
    return x, z

# Параметры шага
L = 1.0  # Длина шага
h = 0.2  # Высота подъёма
t_values = np.linspace(0, 1, 100)

# Вычисление траектории
x_values, z_values = [], []
for t in t_values:
    x, z = foot_trajectory(t, L, h)
    x_values.append(x)
    z_values.append(z)

# Построение графика
plt.figure(figsize=(8, 4))
plt.plot(x_values, z_values, label="Каплеобразная траектория")
plt.xlabel("Горизонтальное положение (x)")
plt.ylabel("Вертикальное положение (z)")
plt.title("Траектория конца ноги шагающего робота")
plt.axhline(0, color='gray', linestyle='--', linewidth=0.5)  # Земля
plt.legend()
plt.grid(True)
plt.show()