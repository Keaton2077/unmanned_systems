import heapq
import numpy as np
import matplotlib.pyplot as plt
import time

class Node:
    def __init__(self, x, y, cost, heuristic):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic
        self.parent = None

    def __lt__(self, other):
        return self.total_cost < other.total_cost

class Obstacle:
    def __init__(self, x_pos: float, y_pos: float, radius: float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius

    def is_inside(self, curr_x: float, curr_y: float) -> bool:
        dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
        return dist_from <= self.radius

def get_all_moves(current_x: float, current_y: float, gs: float) -> list:
    move_list = []
    gs_x_bounds = np.arange(-gs, gs + gs, gs)
    gs_y_bounds = np.arange(-gs, gs + gs, gs)

    for dx in gs_x_bounds:
        for dy in gs_y_bounds:
            x_next = current_x + dx
            y_next = current_y + dy

            if [x_next, y_next] == [current_x, current_y]:
                continue

            move = [x_next, y_next]
            move_list.append(move)

    return move_list

def is_valid_move(obstacle_list: list, x_min: float, y_min: float, x_max: float, y_max: float, x_curr: float, y_curr: float) -> bool:
    if x_min > x_curr or x_max < x_curr or y_min > y_curr or y_max < y_curr:
        return False

    for obs in obstacle_list:
        if obs.is_inside(x_curr, y_curr):
            return False

    return True

def dynamic_a_star(start, goal, dynamic_costs, obstacle_list, robot_radius, gs):
    open_set = [Node(start[0], start[1], 0, heuristic(start, goal))]
    closed_set = set()

    while open_set:
        current_node = heapq.heappop(open_set)

        if (current_node.x, current_node.y) == goal:
            return construct_path(current_node)

        closed_set.add((current_node.x, current_node.y))

        for neighbor in get_neighbors(current_node, dynamic_costs, obstacle_list, robot_radius, gs, goal):
            if (neighbor.x, neighbor.y) in closed_set:
                continue

            tentative_cost = current_node.cost + neighbor.cost
            if (neighbor.x, neighbor.y) not in ((n.x, n.y) for n in open_set) or tentative_cost < neighbor.cost:
                neighbor.cost = tentative_cost
                neighbor.total_cost = tentative_cost + neighbor.heuristic
                neighbor.parent = current_node

                if (neighbor.x, neighbor.y) not in closed_set:
                    heapq.heappush(open_set, neighbor)

    return None

def get_neighbors(node, dynamic_costs, obstacle_list, robot_radius, gs, goal):
    neighbors = []
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    for dx, dy in directions:
        x, y = node.x + dx * gs, node.y + dy * gs
        if is_valid_move(obstacle_list, 0, 0, np.inf, np.inf, x, y):
            cost = 1 + dynamic_costs.get((x, y), 0)
            heuristic_value = heuristic((x, y), goal)
            neighbors.append(Node(x, y, cost, heuristic_value))

    return neighbors

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def construct_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

def visualize_path(obstacle_list, robot_radius, path, start, goal):
    plt.figure(figsize=(8, 8))
    for obstacle in obstacle_list:
        circle = plt.Circle((obstacle.x_pos, obstacle.y_pos), obstacle.radius, color='red')
        plt.gca().add_patch(circle)

    plt.scatter(*start, color='green', marker='o', label='Start')
    plt.scatter(*goal, color='blue', marker='o', label='Goal')

    for i in range(len(path) - 1):
        plt.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], color='black')

    plt.xlim(-1, 20)
    plt.ylim(-1, 20)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.show()

# Example usage:
start_point = (1, 1)
goal_point = (7, 13)
gs = 1
Obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8,
8, 8, 8, 8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
Obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7,
8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,
14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]
obstacle_positions = list(zip(Obstacle_x, Obstacle_y))
obstacle_radius = 0.5
obstacle_list = [Obstacle(obs_pos[0], obs_pos[1], obstacle_radius) for obs_pos in obstacle_positions]
robot_radius = 0.5

start_time = time.time()
path = dynamic_a_star(start_point, goal_point, {}, obstacle_list, robot_radius, gs)
end_time = time.time()
execution_time = end_time - start_time
print("Execution time:", execution_time, "seconds")

if path:
    print("Path found:", path)
    visualize_path(obstacle_list, robot_radius, path, start_point, goal_point)
else:
    print("No path found.")
