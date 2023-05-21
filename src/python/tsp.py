import numpy as np
import math
from python_tsp.exact import solve_tsp_dynamic_programming


def euclidian_distance(tuple1, tuple2):
    x1 = tuple1[0]
    y1 = tuple1[1]
    x2 = tuple2[0]
    y2 = tuple2[1]
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)
def tsp( coordinates):
    distances = []
    for i in range(len(coordinates)):
        arr = []
        for j in range(len(coordinates)):
            arr.append(euclidian_distance(coordinates[i], coordinates[j]))
        distances.append(arr)
    
    distances = np.asarray(distances)
    permutation, distance = solve_tsp_dynamic_programming(distances)

    print(distance)
    print(permutation)
    # Initialize fitness function object using coords_list
    # fitness_coords = mlrose.TravellingSales(coords = coordinates)
    # problem_fit = mlrose.TSPOpt(length = 8, fitness_fn = fitness_coords,
    #                         maximize=False)
    # best_state, best_fitness = mlrose.genetic_alg(problem_fit, random_state = 2)
    # print('The best state found is: ', best_state)

coordinates = coordinates=[(1, 1), (4, 2), (5, 2), (6, 4), (4, 4), (3, 6), (1, 5), (2, 3)]   
tsp(coordinates)