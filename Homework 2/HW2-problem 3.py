import matplotlib.pyplot as plt
import numpy as np
import math as m
import matplotlib.patches as patches

x_array = []
y_array = []

for i in np.arange(0, 10.5, 0.5): 
    x_array.append(i)  
    y_array.append(i + 5)  

plt.plot(x_array, y_array)

def compute_index(min_x: int, max_x: int, min_y: int, max_y: int, gs: float, x_current: int, y_current: int) -> int:
    index = int(((x_current - min_x) / gs) + (((y_current - min_y) / gs) * ((max_x + gs) - min_x) / gs))
    return index

min_x = 0
max_x = 10 

gs = 0.5

min_y = 0
max_y = 10

x_current = 10
y_current = 2

index = compute_index(min_x, max_x, min_y, max_y, gs, x_current, y_current)

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
        index = compute_index(min_x, max_x, min_y, max_y, gs, x, y)
        ax.text(x , y , str(index), color='red', fontsize=6, ha='center', va='center')


ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("")

class Node():
    def __init__(self, x:float ,y:float,cost:float, parent_idx:int) -> None:
        self.x = x
        self.y = y 
        self.cost = cost
        self.parent_idx = int(parent_idx)



class Obstacle():
    def __init__(self, x_pos:float, y_pos:float, radius:float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius
        
    def is_inside(self,curr_x:float, curr_y:float, robot_radius:float=0) -> bool:
        dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
        
        if dist_from > self.radius + robot_radius:
            return False
        
        return True

def is_not_valid(obst_list:list, x_min:int, y_min:int, x_max:int, y_max:int,x_curr:float, y_curr:float, agent_radius:float=0.0):

    for obs in obst_list:
        if obs.is_inside(x_curr, y_curr,agent_radius):
            print("You're dead at ", obs.x_pos, obs.y_pos)
            return True
    
    if x_min > x_curr:
        return True
    if x_max < x_curr:
        return True
    if y_min > y_curr:
        return True
    if y_max < y_curr:
        return True

    return False
    

def get_all_moves(current_x:float, current_y:float, gs:float) -> list:
    move_list = []
    gs_x_bounds = np.arange(-gs, gs+gs, gs)
    gs_y_bounds = np.arange(-gs, gs+gs, gs)
    
    for dx in gs_x_bounds:
        for dy in gs_y_bounds:
            x_next = current_x + dx
            y_next = current_y + dy
            
            if [x_next, y_next] == [current_x, current_y]:
                continue
            
            move = [x_next, y_next]
            move_list.append(move)
            
    return move_list
    
obstacle_positions =  [(1,1), (4,4), (3,4), (5,0)]
obstacle_list = [] 
obstacle_radius = 0.25

# Loop through position of obstacles
for obs_pos in obstacle_positions:
    obstacle = Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)
    obstacle_list.append(obstacle)
    
for obs in obstacle_list:
    obstacle_circle = patches.Circle((obs.x_pos, obs.y_pos), radius=obs.radius, edgecolor='black', facecolor='black')
    ax.add_patch(obstacle_circle)

start_x = 0
start_y = 0

goal_x = 8
goal_y = 9

start_circle = patches.Circle((start_x, start_y), radius=0.3, edgecolor='yellow', facecolor='none')
ax.add_patch(start_circle)
goal_circle = patches.Circle((goal_x, goal_y), radius=0.3, edgecolor='blue', facecolor='none')
ax.add_patch(goal_circle)

unvisited = {}
visited = {}

current_node = Node(start_x, start_y, 0, int(-1))
current_idx = int(compute_index(min_x, max_x, min_y, max_y,gs, start_x, start_y))
unvisited[current_idx] = current_node
while [current_node.x, current_node.y] != [goal_x, goal_y]:
    current_idx = min(unvisited, key=lambda x:unvisited[x].cost)    
    current_node = unvisited[current_idx]
    visited[current_idx] = current_node
    del unvisited[current_idx] 
    
    if [current_node.x, current_node.y] == [goal_x, goal_y]:
        print("Path founded!")
        
        wp_node = current_node
        wp_list = []
        wp_list.append([wp_node.x, wp_node.y])
        
        while wp_node.parent_idx != -1:
            next_idx = wp_node.parent_idx
            wp_node  = visited[next_idx]            
            wp_list.append([wp_node.x, wp_node.y])
        break

    all_moves = get_all_moves(current_node.x, current_node.y, gs)

    filtered_moves = []
    
    for move in all_moves:
        if (is_not_valid(obstacle_list, min_x, min_y, max_x, max_y, move[0], move[1]) == True):
            continue
        else:
            print("good move", move[0], move[1])
            filtered_moves.append(move)
            
    #  - loop through all filtered moves:
    for move in filtered_moves:
        new_index = int(compute_index(min_x, max_x, min_y, max_y,gs, move[0], move[1]))
        
        new_cost = current_node.cost + m.dist(move, [current_node.x, current_node.y])

        if new_index in visited:
            continue
      
        if new_index in unvisited:
            if new_cost < unvisited[new_index].cost:
                unvisited[new_index].cost = new_cost
                unvisited[new_index].parent_idx = current_idx
                
            continue    
        
        new_node = Node(move[0], move[1], new_cost, current_idx)
        unvisited[new_index] = new_node 
        
path_x = [point[0] for point in wp_list]
path_y = [point[1] for point in wp_list]

plt.plot(path_x, path_y, marker='o', color='green', linestyle='-')
