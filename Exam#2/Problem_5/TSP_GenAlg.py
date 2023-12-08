import matplotlib.pyplot as plt
import math as m
import numpy as np
import matplotlib.pyplot as plt
import time

# 配送点
cities = np.array([(9,4), (4,4), (1,9), (9,7), (6,14)])

# 障碍物列表
obstacle_list = np.array([(2,2), (2,3), (2,4), (2,5), (0,5), (1,5), (2,5), (3,5), (4,5), (5,5), (8,2), (9,2), (10,2), (11,2),
(12,2), (13,3), (8,4), (8,5), (8,6), (8,7), (8,8), (8,9), (8,7), (2,7), (3,7), (4,7), (5,7), (6,7), (7,6), (9,6),
(10,6), (11,6), (12,6), (15,8), (2,9), (2,10), (2,11), (2,12), (2,13), (5,9), (5,10), (5,11), (5,12), (5,13),
(5,14), (5,15), (6,12), (7,12), (8,12), (9,12), (10,12), (11,12), (12,8), (12,9), (12,10), (12,11), (12,12)])

adjacency_mat = np.asarray(
    [
        [0.00, 28.02, 17.12, 27.46, 46.07],
        [28.02, 0.00, 34.00, 25.55, 25.55],
        [17.12, 34.00, 0.00, 18.03, 57.38],
        [27.46, 25.55, 18.03, 0.00, 51.11],
        [46.07, 25.55, 57.38, 51.11, 0.00],
    ]
)

class Population():
    def __init__(self, bag, adjacency_mat):
        self.bag = bag
        self.parents = []
        self.score = 0
        self.best = None
        self.adjacency_mat = adjacency_mat

def init_population(cities, adjacency_mat, n_population):
    return Population(
        np.asarray([np.random.permutation(cities) for _ in range(n_population)]), 
        adjacency_mat
    )

def fitness(self, chromosome):
    return sum(
        [
            self.adjacency_mat[chromosome[i], chromosome[i + 1]]
            for i in range(len(chromosome) - 1)
        ]
    )

Population.fitness = fitness

def evaluate(self):
    distances = np.asarray(
        [self.fitness(chromosome) for chromosome in self.bag]
    )
    self.score = np.min(distances)
    self.best = self.bag[distances.tolist().index(self.score)]
    self.parents.append(self.best)
    if False in (distances[0] == distances):
        distances = np.max(distances) - distances
    return distances / np.sum(distances)
    
Population.evaluate = evaluate

def select(self, k=4):
    fit = self.evaluate()
    while len(self.parents) < k:
        idx = np.random.randint(0, len(fit))
        if fit[idx] > np.random.rand():
            self.parents.append(self.bag[idx])
    self.parents = np.asarray(self.parents)

Population.select = select

def swap(chromosome):
    a, b = np.random.choice(len(chromosome), 2)
    chromosome[a], chromosome[b] = (
        chromosome[b],
        chromosome[a],
    )
    return chromosome

def crossover(self, p_cross=0.1):
    children = []
    count, size = self.parents.shape
    for _ in range(len(self.bag)):
        if np.random.rand() > p_cross:
            children.append(
                list(self.parents[np.random.randint(count, size=1)[0]])
            )
        else:
            parent1, parent2 = self.parents[
                np.random.randint(count, size=2), :
            ]
            idx = np.random.choice(range(size), size=2, replace=False)
            start, end = min(idx), max(idx)
            child = [None] * size
            for i in range(start, end + 1, 1):
                child[i] = parent1[i]
            pointer = 0
            for i in range(size):
                if child[i] is None:
                    while parent2[pointer] in child:
                        pointer += 1
                    child[i] = parent2[pointer]
            children.append(child)
    return children

Population.crossover = crossover

def mutate(self, p_cross=0.1, p_mut=0.1):
    next_bag = []
    children = self.crossover(p_cross)
    for child in children:
        if np.random.rand() < p_mut:
            next_bag.append(swap(child))
        else:
            next_bag.append(child)
    return next_bag
    
Population.mutate = mutate

def genetic_algorithm(
    cities,
    adjacency_mat,
    n_population=500,
    n_iter=2000,
    selectivity=0.15,
    p_cross=0.5,
    p_mut=0.1,
    print_interval=100,
    return_history=False,
    verbose=False,
):
    # 从城市列表中移除第一个城市
    cities = cities[1:]
    
    pop = init_population(cities, adjacency_mat, n_population)
    best = pop.best
    score = float("inf")
    history = []
    
    for i in range(n_iter):
        pop.select(n_population * selectivity)
        history.append(pop.score)
        if verbose:
            print(f"Generation {i}: {pop.score}")
        elif i % print_interval == 0:
            print(f"Generation {i}: {pop.score}")
        if pop.score < score:
            best = pop.best
            score = pop.score
        children = pop.mutate(p_cross, p_mut)
        pop = Population(children, pop.adjacency_mat)
    
    # 在最佳路径前面添加第一个城市
    best = np.insert(best, 0, 0)
    
    if return_history:
        return best, history
    return best

# 计算运行时间
start_time = time.time()
best_path = genetic_algorithm(cities, adjacency_mat, verbose=True)
end_time = time.time()
print(f"Computation time: {end_time - start_time} seconds")

# 计算最佳路径成本
best_path_cost = sum(
    [
        adjacency_mat[best_path[i], best_path[i + 1]]
        for i in range(len(best_path) - 1)
    ]
)
print(f"Best path cost: {best_path_cost}")

# 绘制障碍物和路径
plt.figure()
plt.plot(obstacle_list[:, 0], obstacle_list[:, 1], "ro")
plt.plot(cities[:, 0], cities[:, 1], "bo")
plt.plot(best_path[:, 0], best_path[:, 1], "g-")
plt.show()
