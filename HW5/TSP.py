import itertools
import matplotlib.pyplot as plt
import numpy as np

# Define the waypoints
waypoints = [(0, 0), (2, 2), (5, 3), (3, 4), (6, 4)]

# Compute the distance between all waypoints and store it in a lookup table
lookup_table = {}
for i in range(len(waypoints)):
    for j in range(len(waypoints)):
        if i != j:
            distance = np.sqrt((waypoints[i][0] - waypoints[j][0])**2 + (waypoints[i][1] - waypoints[j][1])**2)
            lookup_table[(i, j)] = distance

# Generate all possible permutations of the waypoints
permutations = list(itertools.permutations(range(1, len(waypoints))))

# Initialize variables for the minimum cost and best path
min_cost = float('inf')
best_path = None

# Compute the total cost for each permutation and find the minimum
for perm in permutations:
    current_path = [0] + list(perm) + [0]  # Append start and end points
    total_cost = sum(lookup_table[(current_path[k], current_path[k + 1])] for k in range(len(current_path) - 1))
    if total_cost < min_cost:
        min_cost = total_cost
        best_path = current_path

# Plot the optimal path
optimal_path = [waypoints[i] for i in best_path]
x_coords = [point[0] for point in optimal_path]
y_coords = [point[1] for point in optimal_path]

plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b')
for i, txt in enumerate(best_path):
    plt.annotate(txt, (x_coords[i], y_coords[i]), textcoords="offset points", xytext=(0,10), ha='center')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Optimal Path for TSP')
plt.show()
