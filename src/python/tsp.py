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

    ans = []
    for i in permutation:
        if i != 0:
            ans.append((coordinates[i][0], coordinates[i][1], coordinates[i][2], coordinates[i][3], coordinates[i][4]))
    # print("answer: ", ans)
    return ans, distance
    # Initialize fitness function object using coords_list
    # fitness_coords = mlrose.TravellingSales(coords = coordinates)
    # problem_fit = mlrose.TSPOpt(length = 8, fitness_fn = fitness_coords,
    #                         maximize=False)
    # best_state, best_fitness = mlrose.genetic_alg(problem_fit, random_state = 2)
    # print('The best state found is: ', best_state)


# coordinates =[(0, 0), (2.4, 3.0, 'Cracker', 1, 1), (2.4, 2.0, 'Sugar', 1, 2), (2.4, 1.0, 'Soup', 1, 3), (-2.4, 2.0, 'Jello', 1, 5), (-2.4, 3.0, 'Mustard', 1, 4), (-2.4, 1.0, 'Meat', 1, 6)]
# coordinates =[(0, 0), (2.4, 2.0, 'Sugar', 1, 2), (2.4, 3.0, 'Cracker', 1, 1), (2.4, 1.0, 'Soup', 1, 3), (-2.4, 2.0, 'Jello', 1, 5), (-2.4, 3.0, 'Mustard', 1, 4), (-2.4, 1.0, 'Meat', 1, 6)]
# tsp(coordinates)
