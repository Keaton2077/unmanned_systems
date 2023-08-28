#Problem 4
import matplotlib.pyplot as plt
import numpy as np

# Creating a Coordinate System
grid_size = 0.5
min_x, max_x = 0, 10
min_y, max_y = 0, 10
x_values = np.arange(min_x, max_x + grid_size, grid_size)
y_values = np.arange(min_y, max_y + grid_size, grid_size)
fig, ax = plt.subplots()
ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
for x in x_values:
    plt.axvline(x=x, color='gray', linewidth=0.5)
for y in y_values:
    plt.axhline(y=y, color='gray', linewidth=0.5)

# Compute and plot node indices
node_index = 0
for y in y_values:
    for x in x_values:
        plt.text(x + grid_size / 2, y + grid_size / 2, str(node_index), color='red', fontsize=6, ha='center', va='center')
        node_index += 1

plt.gca().set_aspect('equal', adjustable='box')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Node Index')
plt.grid(True)
plt.show()




