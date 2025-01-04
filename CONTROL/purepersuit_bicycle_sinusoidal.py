import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, atan2

x_path = np.linspace(0, 10, 1000)
y_path = 5 * np.sin(x_path)

L = 1.0  
speed = 0.1  
lookahead_dist = 1.0  

x, y, theta = 0, 0, 0 
rx, ry = [x], [y]

for i in range(1, len(x_path)):
 
    dist_to_target = np.sqrt((x_path[i] - x)**2 + (y_path[i] - y)**2)
    

    if dist_to_target >= lookahead_dist:
        target_x, target_y = x_path[i], y_path[i]
        alpha = atan2(target_y - y, target_x - x) - theta
        delta = atan2(2 * L * sin(alpha) / lookahead_dist, 1)
        x += speed * cos(theta)
        y += speed * sin(theta)
        theta += delta

        rx.append(x)
        ry.append(y)

    if i == len(x_path) - 1:
        break

plt.plot(x_path, y_path, color='black', label="Path") 
plt.plot(rx, ry, color='red', label="vehicle")
plt.title("Pure Pursuit Bicycle Following Sinusoidal Path")
plt.legend()
plt.show()
