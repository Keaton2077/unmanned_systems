#Problem 4
import numpy as np
import matplotlib.pyplot as plt

x_array = []
y_array = []

for i in np.arange(0, 10.5, 0.5): 
    x_array.append(i)  
    y_array.append(i + 5)  

plt.plot(x_array, y_array)

def compile_index(min_x: int, max_x: int, min_y: int, max_y: int, gs: float, x_current: int, y_current: int) -> int:
    index = int(((x_current - min_x) / gs) + (((y_current - min_y) / gs) * ((max_x + gs) - min_x) / gs))
    return index

min_x = 0
max_x = 10 
gs = 0.5

min_y = 0
max_y = 10

x_current = 10
y_current = 2

index = compile_index(min_x, max_x, min_y, max_y, gs, x_current, y_current)

x_values = np.arange(min_x, max_x + gs, gs)
y_values = np.arange(min_y, max_y + gs, gs)

fig, ax = plt.subplots()
ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)
for x in x_values:
    plt.axvline(x=x, color='gray', linewidth=0.5)
for y in y_values:
    plt.axhline(y=y, color='gray', linewidth=0.5)

for x in x_values:
    for y in y_values:
        index = compile_index(min_x, max_x, min_y, max_y, gs, x, y)
        ax.text(x , y , str(index), color='red', fontsize=8, ha='center', va='center')

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("Grid Node Indices")



