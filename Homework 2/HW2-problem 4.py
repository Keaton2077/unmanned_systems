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
        self.radius = radius + (robot_diameter / 2)
        
    def is_inside(self, curr_x: float, curr_y: float, robot_radius: float = 0) -> bool:
        dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
        return dist_from <= self.radius + robot_radius

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

def find_path(start_point, goal_point, gs, obstacle_positions, obstacle_radius):
    min_x, max_x = 0, 10 
    min_y, max_y = 0, 10 
    
    fig, ax = plt.subplots()
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    x_values = np.arange(min_x, max_x + gs, gs)
    y_values = np.arange(min_y, max_y + gs, gs)
    for x in x_values:
        plt.axvline(x=x, color='gray', linewidth=0.5)
    for y in y_values:
        plt.axhline(y=y, color='gray', linewidth=0.5)
    
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

    unvisited = {}
    visited = {}

    start_node = Node(start_point[0], start_point[1], 0, -1)
    start_index = compute_index(min_x, max_x, min_y, max_y, gs, start_point[0], start_point[1])
    unvisited[start_index] = start_node

    while [start_node.x, start_node.y] != [goal_point[0], goal_point[1]]:
        current_index = min(unvisited, key=lambda x: unvisited[x].cost)
        current_node = unvisited[current_index]
        visited[current_index] = current_node
        del unvisited[current_index] 

        if [current_node.x, current_node.y] == [goal_point[0], goal_point[1]]:
            print("Path founded!")

            wp_node = current_node
            wp_list = []
            wp_list.append([wp_node.x, wp_node.y])

            while wp_node.parent_idx != -1:
                next_idx = wp_node.parent_idx
                wp_node = visited[next_idx]            
                wp_list.append([wp_node.x, wp_node.y])
            break

        possible_moves = get_all_moves(current_node.x, current_node.y, gs)
        valid_moves = [move for move in possible_moves if is_valid_move(obstacle_list, min_x, min_y, max_x, max_y, move[0], move[1])]

        for move in valid_moves:
            new_index = compute_index(min_x, max_x, min_y, max_y, gs, move[0], move[1])
            new_cost = current_node.cost + m.dist(move, [current_node.x, current_node.y])

            if new_index in visited:
                continue

            if new_index in unvisited:
                if new_cost < unvisited[new_index].cost:
                    unvisited[new_index].cost = new_cost
                    unvisited[new_index].parent_idx = current_index
                continue

            new_node = Node(move[0], move[1], new_cost, current_index)
            unvisited[new_index] = new_node 

    path_x = [point[0] for point in wp_list]
    path_y = [point[1] for point in wp_list]

    plt.plot(path_x, path_y, marker='o', color='green', linestyle='-')
    plt.show()

# 调用函数查找路径
start_point = (0, 0)
goal_point = (8, 9)
gs = 0.5
obstacle_positions = [(1, 1), (4, 4), (3, 4), (5, 0), (5, 1), (0, 7), (1, 7), (2, 7), (3, 7)]
obstacle_radius = 0.25
robot_diameter = 1.0

obstacle_list = [Obstacle(obs_pos[0], obs_pos[1], obstacle_radius) for obs_pos in obstacle_positions]

find_path(start_point, goal_point, gs, obstacle_positions, obstacle_radius)
