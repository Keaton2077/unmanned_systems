import matplotlib.pyplot as plt
import numpy as np
import math as m
import matplotlib.patches as patches

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

def euclidean_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

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

def compute_index(min_x: float, max_x: float, min_y: float, max_y: float, gs: float, x_current: float, y_current: float) -> int:
    index = int(((x_current - min_x) / gs) + (((y_current - min_y) / gs) * ((max_x + gs) - min_x) / gs))
    return index

def rrt(start_point, goal_point, gs, obstacle_positions, obstacle_radius, max_iter=1000, step_size=0.5):
    min_x, max_x = 0, 10
    min_y, max_y = 0, 10
    
    fig, ax = plt.subplots()
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    
    for x in np.arange(min_x, max_x + gs, gs):
        for y in np.arange(min_y, max_y + gs, gs):
            index = compute_index(min_x, max_x, min_y, max_y, gs, x, y)
            ax.text(x, y, str(index), color='red', fontsize=6, ha='center', va='center')

    for obs_pos in obstacle_positions:
        obstacle = Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)
        obstacle_circle = patches.Circle((obstacle.x_pos, obstacle.y_pos), radius=obstacle.radius, edgecolor='black', facecolor='black')
        ax.add_patch(obstacle_circle)

    start_circle = patches.Circle((start_point[0], start_point[1]), radius=0.3, edgecolor='yellow', facecolor='none')
    ax.add_patch(start_circle)
    
    goal_circle = patches.Circle((goal_point[0], goal_point[1]), radius=0.3, edgecolor='blue', facecolor='none')
    ax.add_patch(goal_circle)

    tree = {start_point: None}

    for _ in range(max_iter):
        random_point = (np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y))
        nearest_point = min(tree, key=lambda p: euclidean_distance(p, random_point))
        
        # Move towards the random point with step size
        delta_x = random_point[0] - nearest_point[0]
        delta_y = random_point[1] - nearest_point[1]
        distance = euclidean_distance(nearest_point, random_point)
        if distance > step_size:
            scale = step_size / distance
            new_point = (nearest_point[0] + delta_x * scale, nearest_point[1] + delta_y * scale)
        else:
            new_point = random_point
        
        if is_valid_move(obstacle_list, min_x, min_y, max_x, max_y, new_point[0], new_point[1]):
            tree[new_point] = nearest_point
            if euclidean_distance(new_point, goal_point) <= step_size:
                tree[goal_point] = new_point
                break

    # Reconstruct the path
    path = [goal_point]
    while path[-1] != start_point:
        path.append(tree[path[-1]])

    path.reverse()
    path_x = [point[0] for point in path]
    path_y = [point[1] for point in path]

    # Plot the tree
    for point, parent in tree.items():
        if parent is not None:
            plt.plot([point[0], parent[0]], [point[1], parent[1]], color='gray', linewidth=0.5)

    plt.plot(path_x, path_y, marker='o', color='green', linestyle='-')
    plt.show()

start_point = (0, 0)
goal_point = (8, 9)
gs = 0.5
obstacle_positions = [(1, 1), (4, 4), (3, 4), (5, 0), (5, 1), (0, 7), (1, 7), (2, 7), (3, 7)]
obstacle_radius = 0.25

obstacle_list = [Obstacle(obs_pos[0], obs_pos[1], obstacle_radius) for obs_pos in obstacle_positions]

# Use the rrt function to find the path using RRT
rrt(start_point, goal_point, gs, obstacle_positions, obstacle_radius)
