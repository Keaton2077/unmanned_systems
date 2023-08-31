# -*- coding: utf-8 -*-
"""
Created on Sun Aug 27 21:21:17 2023

@author: Aiden
"""

#Problem 1

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.ticker as ticker

# Creating a Coordinate System
fig, ax = plt.subplots()
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.xaxis.set_major_locator(ticker.MultipleLocator(base=1))
ax.yaxis.set_major_locator(ticker.MultipleLocator(base=1))
plt.grid(True, linestyle='--', linewidth=0.5, color='gray')

# Draw obstacles
polygon_vertices = [
    [(1, 2), (3, 2), (3, 4)],
    [(2, 8), (6, 1), (5, 6), (6, 7)],
    [(8, 6), (6, 9), (8, 8)]
]

for vertices in polygon_vertices:
    polygon = patches.Polygon(vertices, closed=True, edgecolor='black', facecolor='red')
    ax.add_patch(polygon)
    
# Draw start and end points
circle1 = patches.Circle((1, 1), radius=0.3, edgecolor='black', facecolor='none')
ax.add_patch(circle1)
circle2 = patches.Circle((9, 8), radius=0.3, edgecolor='blue', facecolor='none')
ax.add_patch(circle2)

#Draw reduced graph
plt.plot([1, 6], [1, 1], color='blue')
plt.plot([6, 9], [1, 8], color='blue')

#Draw remaining standard edges
plt.plot([1, 2], [2, 8], color='black')
plt.plot([2, 6], [8, 9], color='black')
plt.plot([6, 9], [9, 8], color='black')
plt.plot([1, 1], [1, 2], color='black')
plt.plot([1, 3], [2, 2], color='black')
plt.plot([3, 6], [2, 1], color='black')
plt.plot([3, 3], [2, 4], color='black')
plt.plot([1, 3], [2, 4], color='black')
plt.plot([3, 2], [4, 8], color='black')
plt.plot([3, 6], [4, 1], color='black')
plt.plot([6, 6], [1, 9], color='black')
plt.plot([6, 8], [1, 6], color='black')
plt.plot([2, 6], [8, 7], color='black')
plt.plot([6, 8], [7, 6], color='black')
plt.plot([8, 9], [8, 8], color='black')
plt.plot([2, 6], [8, 1], color='black')
plt.plot([6, 5], [1, 6], color='black')
plt.plot([5, 6], [6, 7], color='black')
plt.plot([8, 9], [8, 8], color='black')
plt.plot([6, 8], [9, 6], color='black')
plt.plot([6, 8], [9, 8], color='black')
plt.plot([8, 8], [8, 6], color='black')

#Problem 2
class Node:
    def __init__(self, x, y, parent_cost, index):
        self.x = x
        self.y = y
        self.parent_cost = parent_cost
        self.index = index


#Problem 3
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.ticker as ticker
import numpy as np

# Creating a Coordinate System
fig, ax = plt.subplots()
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.xaxis.set_major_locator(ticker.MultipleLocator(base=1))
ax.yaxis.set_major_locator(ticker.MultipleLocator(base=1))
plt.grid(True, linestyle='--', linewidth=0.5, color='gray')

# Draw obstacles
polygon_vertices = [(2,8), (6,1), (5,6),(6,7)]
polygon = patches.Polygon(polygon_vertices, closed=True, edgecolor='black', facecolor='red')
ax.add_patch(polygon)

# Draw start and end points
circle1 = patches.Circle((1, 1), radius=0.3, edgecolor='black', facecolor='none')
ax.add_patch(circle1)
circle2 = patches.Circle((9, 8), radius=0.3, edgecolor='blue', facecolor='none')
ax.add_patch(circle2)

# Draw reduced visibility graph
lines1 = [ ([1, 6], [1, 1]), ([6, 9], [1, 8]) ]
for line1 in lines1:
    plt.plot(line1[0], line1[1], color='blue')
    
# Draw remaining standard edges
lines2 = [([2, 6], [8, 1]), ([5, 6], [6, 7]), ([2, 6], [8, 7]), ([6, 9], [7, 8]), ([5, 9], [6, 8]),([5, 6], [6, 1]),([1, 2], [1, 8]),([2, 9], [8, 8])]
for line2 in lines2:
    plt.plot(line2[0], line2[1], color='black')
    
# Calculate Euclidean distance and display
lines = lines1 + lines2
for i, line in enumerate(lines):
    start_point = np.array([line[0][0], line[1][0]])
    end_point = np.array([line[0][1], line[1][1]])
    mid_point = (start_point + end_point) / 2
    euclidean_distance = np.linalg.norm(end_point - start_point)
    plt.text(mid_point[0], mid_point[1], f'{euclidean_distance:.2f}', fontsize=12, color='black')
    
