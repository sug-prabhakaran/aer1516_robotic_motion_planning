import numpy as np
import time
import matplotlib.pyplot as plt
import dubins_path_problem as dpp
import seaborn as sns

def main():
    print("Executing: " + __file__)
    # ====Search Path with RRT and RRT*====
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
    
    rrt_cost_list = []      # initialize to store final path costs
    rrt_time_list = []      # initialize to store path calc. times
    rrt_not_found_list = [] # initialize to store iters with path not found
    np.random.seed(1)       # seed value

    # solve 1000 random instances of new map using RRT
    for i in range(100):
        start_time = time.time()
        path_node_list = rrt_dubins.rrt_planning(display_map=False)
        elapsed_time = time.time()-start_time
        if path_node_list:
            print("iter: ", i, "distance:", path_node_list[-1].cost)
            rrt_cost_list.append(path_node_list[-1].cost)
            rrt_time_list.append(elapsed_time)
            
            # plot the last graph for report
            if i == 999:
                path = dpp.get_path(path_node_list)
                rrt_dubins.draw_graph()
                plt.figure(1)
                plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                plt.grid(True)
                plt.pause(0.001) # Necessary for macs
                #plt.show()
        else:
            print("iter: ", i, "not found")
            rrt_not_found_list.append(i)

    #print("cost:", rrt_cost)

    #Gather Data
    rrt_avg_cost = sum(rrt_cost_list)/len(rrt_cost_list)
    rrt_avg_time = sum(rrt_time_list)/len(rrt_time_list)
    num_rrt_failed = len(rrt_not_found_list)
    print(num_rrt_failed)

    #Plot Figure
    plt.style.use('ggplot')
    fig, ax = plt.subplots(num=2, figsize=(8,5))

    ax.bar([0],[rrt_avg_cost], color='#0485d1', label='RRT', width=0.3)
    ax.set_ylabel('Avg. cost to goal')
    ax.set_xlabel('Comparison of RRT vs. RRT*')
    ax.legend(facecolor='white')
    ax.set_title("Comparison of Cost-to-goal and Calculation Time for RRT and RRT*")
    ax2= ax.twinx()
    ax2.bar([1], [rrt_avg_time], color='#0485d1', label='RRT', width=0.3)
    ax2.set_ylabel('Avg. time to calc. path')
    ax2.set_yticks(np.arange(0,0.021, 0.002))
    ax2.grid(None)
    plt.show()

if __name__ == '__main__':
    main()