import matplotlib.pyplot as plt
import numpy as np
import math as m
import matplotlib.patches as patches
import time
import itertools

class Node:
    def __init__(self, x: float, y: float, cost: float, parent_idx: int) -> None:
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_idx = parent_idx

class Obstacle:
    def __init__(self, x_pos: float, y_pos: float, radius: float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius

    def is_inside(self, curr_x: float, curr_y: float) -> bool:
        dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
        return dist_from <= self.radius

class Astar:
    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float, gs: float, obstacle_positions: list, obstacle_radius: float):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.gs = gs
        self.obstacle_positions = obstacle_positions
        self.obstacle_radius = obstacle_radius
        self.obstacle_list = [Obstacle(obs_pos[0], obs_pos[1], obstacle_radius) for obs_pos in obstacle_positions]

    def get_all_moves(self, current_x: float, current_y: float) -> list:
        move_list = []
        gs_x_bounds = np.arange(-self.gs, self.gs + self.gs, self.gs)
        gs_y_bounds = np.arange(-self.gs, self.gs + self.gs, self.gs)

        for dx in gs_x_bounds:
            for dy in gs_y_bounds:
                x_next = current_x + dx
                y_next = current_y + dy

                if [x_next, y_next] == [current_x, current_y]:
                    continue

                move = [x_next, y_next]
                move_list.append(move)

        return move_list

    def is_valid_move(self, x_curr: float, y_curr: float) -> bool:
        if self.min_x > x_curr or self.max_x < x_curr or self.min_y > y_curr or self.max_y < y_curr:
            return False

        for obs in self.obstacle_list:
            if obs.is_inside(x_curr, y_curr):
                return False

        return True

    def compute_index(self, x_current: float, y_current: float) -> int:
        index = int(((x_current - self.min_x) / self.gs) + (((y_current - self.min_y) / self.gs) * ((self.max_x + self.gs) - self.min_x) / self.gs))
        return index

    def find_path(self, start_point, goal_point):
        unvisited = {}
        visited = {}

        start_node = Node(start_point[0], start_point[1], 0, -1)
        start_index = self.compute_index(start_point[0], start_point[1])
        unvisited[start_index] = start_node

        while [start_node.x, start_node.y] != [goal_point[0], goal_point[1]]:
            current_index = min(unvisited, key=lambda x: unvisited[x].cost)
            current_node = unvisited[current_index]
            visited[current_index] = current_node
            del unvisited[current_index]

            if [current_node.x, current_node.y] == [goal_point[0], goal_point[1]]:
                wp_node = current_node
                wp_list = []
                wp_list.append([wp_node.x, wp_node.y])

                while wp_node.parent_idx != -1:
                    next_idx = wp_node.parent_idx
                    wp_node = visited[next_idx]
                    wp_list.append([wp_node.x, wp_node.y])
                break

            possible_moves = self.get_all_moves(current_node.x, current_node.y)
            valid_moves = [move for move in possible_moves if self.is_valid_move(move[0], move[1])]

            for move in valid_moves:
                new_index = self.compute_index(move[0], move[1])
                new_cost = current_node.cost + m.dist(move, [current_node.x, current_node.y]) + m.dist(move, goal_point)

                if new_index in visited:
                    continue

                if new_index in unvisited:
                    if new_cost < unvisited[new_index].cost:
                        unvisited[new_index].cost = new_cost
                        unvisited[new_index].parent_idx = current_index
                    continue

                new_node = Node(move[0], move[1], new_cost, current_index)
                unvisited[new_index] = new_node

        total_travel_cost = 0
        for i in range(len(wp_list) - 1):
            total_travel_cost += m.dist(wp_list[i], wp_list[i + 1])

        return wp_list, total_travel_cost

# Usage
start_point = (0, 0)
delivery_points = [(9,4), (4,4), (1,9), (9,7), (6,14)]
gs = 0.5
obstacle_positions = [(2,2), (2,3), (2,4), (2,5), (0,5), (1,5), (2,5), (3,5), (4,5), (5,5), (8,2), (9,2), (10,2), (11,2),
(12,2), (13,3), (8,4), (8,5), (8,6), (8,7), (8,8), (8,9), (8,7), (2,7), (3,7), (4,7), (5,7), (6,7), (7,6), (9,6),
(10,6), (11,6), (12,6), (15,8), (2,9), (2,10), (2,11), (2,12), (2,13), (5,9), (5,10), (5,11), (5,12), (5,13),
(5,14), (5,15), (6,12), (7,12), (8,12), (9,12), (10,12), (11,12), (12,8), (12,9), (12,10), (12,11), (12,12)]
obstacle_radius = 0.5

astar = Astar(0, 15, 0, 15, gs, obstacle_positions, obstacle_radius)
delivery_paths = []
for delivery_point in delivery_points:
    path, _ = astar.find_path(start_point, delivery_point)
    delivery_paths.append(path)
    start_point = delivery_point

# Plotting
fig, ax = plt.subplots()
ax.set_xlim(0, 15)
ax.set_ylim(0, 15)

for obs_pos in obstacle_positions:
    obstacle = patches.Circle((obs_pos[0], obs_pos[1]), radius=obstacle_radius, edgecolor='black', facecolor='black')
    ax.add_patch(obstacle)

for path in delivery_paths:
    path_x = [point[0] for point in path]
    path_y = [point[1] for point in path]
    plt.plot(path_x, path_y, marker='o', color='green', linestyle='-')

plt.show()
