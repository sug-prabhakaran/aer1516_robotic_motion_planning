import numpy as np
import matplotlib.pyplot as plt

import dubins_path_problem as dpp
from rrt_planner import rrt_planner
from rrt_star_planner import rrt_star_planner

def main():
    print("Executing: " + __file__)
    # ====Search Path with RRT====
    obstacleList = [
        (2, 10, 1),
        (4, 7.5, 1.5),
        (6, 5, 1),
        (10, 4, 2),
        (13, 7, 1),
        (0, 13.5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = [0.0, 0.0, np.deg2rad(-50.0)]
    goal = [10.0, 10.0, np.deg2rad(50.0)]

    rrt_dubins = dpp.RRT_dubins_problem(start = start, goal = goal, \
                                    obstacle_list = obstacleList, \
                                    map_area = [-2.0, 15.0, -2.0, 15.0], \
                                    max_iter=100)
    
    rrt_cost = []            # initialize list to store final path costs
    rrt_not_found = []
    # solve 1000 random instances of new map using RRT
    np.random.seed(1)
    for i in range(1000):
        path_node_list = rrt_dubins.rrt_planning(display_map=False)
        if path_node_list:
            print("iter: ", i, "distance:", path_node_list[-1].cost)
            rrt_cost.append(path_node_list[-1].cost)
            path = dpp.get_path(path_node_list)
            if i == 999:
                rrt_dubins.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                plt.grid(True)
                plt.pause(0.001) # Necessary for macs
                plt.show()
        else:
            print("iter: ", i, "not found")
            rrt_not_found.append(i)

    print("cost:", rrt_cost)
    

if __name__ == '__main__':
    main()