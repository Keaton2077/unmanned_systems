import math
import numpy as np
#1)
def euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance

# Test the function with (2, 1) and (3, 2)
point1 = (2, 1)
point2 = (3, 2)
distance = euclidean_distance(point1, point2)

print(f"The Euclidean distance between {point1} and {point2} is {distance}")

#2)
class Obstacle:
    def __init__(self, x_pos: float, y_pos: float, diameter: float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.diameter = diameter
        
    def is_inside(self, curr_x: float, curr_y: float) -> bool:
        dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
        return dist_from <= self.diameter / 2.0

def is_valid_node(obstacle_list, current_node, min_x, max_x, min_y, max_y):
    for obs in obstacle_list:
        if obs.is_inside(current_node[0], current_node[1]):
            return False

    if (min_x <= current_node[0] <= max_x and min_y <= current_node[1] <= max_y):
        return True  

    return False

obstacle_positions = [(1, 1), (4, 4), (3, 4), (5, 0), (5, 1), (0, 7), (1, 7), (2, 7), (3, 7)]
obstacle_list = []

obstacle_diameter = 0.5

for obs_pos in obstacle_positions:
    obstacle = Obstacle(obs_pos[0], obs_pos[1], obstacle_diameter)
    obstacle_list.append(obstacle)

current_node = (2, 2)

min_x = 0
max_x = 10
min_y = 0
max_y = 10

is_node_valid = is_valid_node(obstacle_list, current_node, min_x, max_x, min_y, max_y)

if is_node_valid:
    print("The current location is valid.")
else:
    print("The current location is not valid due to obstacles or being outside the grid boundaries.")
