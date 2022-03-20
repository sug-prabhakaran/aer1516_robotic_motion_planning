"""
Assignment #2 Template file
"""
import random
import math
import numpy as np

"""
Problem Statement
--------------------
Implement the planning algorithm called Rapidly-Exploring Random Trees* (RRT)
for the problem setup given by the RRT_DUBINS_PROMLEM class.

INSTRUCTIONS
--------------------
1. The only file to be submitted is this file rrt_star_planner.py. Your
   implementation can be tested by running RRT_DUBINS_PROBLEM.PY (check the 
   main function).
2. Read all class and function documentation in RRT_DUBINS_PROBLEM carefully.
   There are plenty of helper function in the class to ease implementation.
3. Your solution must meet all the conditions specified below.
4. Below are some do's and don'ts for this problem as well.

Conditions
-------------------
There are some conditions to be satisfied for an acceptable solution.
These may or may not be verified by the marking script.

1. The solution loop must not run for more that a certain number of random iterations
   (Specified by a class member called MAX_ITER). This is mainly a safety
   measure to avoid time-out-related issues and will be set generously.
2. The planning function must return a list of nodes that represent a collision-free path
   from start node to the goal node. The path states (path_x, path_y, path_yaw)
   specified by each node must define a Dubins-style path and traverse from node i-1 -> node i.
   (READ the documentation for the node class to understand the terminology)
3. The returned path should have the start node at index 0 and goal node at index -1,
   while the parent node for node i from the list should be node i-1 from the list, ie,
   the path should be a valid list of nodes.
   (READ the documentation of the node to understand the terminology)
4. The node locations must not lie outside the map boundaries specified by
   RRT_DUBINS_PROBLEM.map_area.

DO(s) and DONT(s)
-------------------
1. Do not rename the file rrt_star_planner.py for submission.
2. Do not change change the PLANNING function signature.
3. Do not import anything other than what is already imported in this file.
4. You can write more function in this file in order to reduce code repitition
   but these function can only be used inside the PLANNING function.
   (since only the planning function will be imported)
"""

def rrt_star_planner(rrt_dubins, display_map=False):
    """
        Execute RRT* planning using Dubins-style paths. Make sure to populate the node_list.

        Inputs
        -------------
        rrt_dubins  - (RRT_DUBINS_PROBLEM) Class conatining the planning
                      problem specification
        display_map - (boolean) flag for animation on or off (OPTIONAL)

        Outputs
        --------------
        (list of nodes) This must be a valid list of connected nodes that form
                        a path from start to goal node

        NOTE: In order for rrt_dubins.draw_graph function to work properly, it is important
        to populate rrt_dubins.nodes_list with all valid RRT nodes.
    """
    # LOOP for max iterations
    rrt_dubins.node_list = [rrt_dubins.start]                     # reset node_list for each iteration
    path_node_list = []                                           # initialize to clear for multiple runs
    i = 0
    while i < rrt_dubins.max_iter:
        i += 1

        # 1. Generate a random vehicle state (x, y, yaw)
        if i % 99 == 0:                                           # every 15 check goal state
            new_node = rrt_dubins.goal
        else:
            position = np.around(17*np.random.rand(2)-2,1)            # random x, y between (-2, 15)
            heading = np.deg2rad(np.around(360*np.random.rand(1),1))  # random theta between (0, 360)
            rand_state = np.concatenate([position, heading])
            new_node = rrt_dubins.Node(rand_state[0], rand_state[1], rand_state[2])

        # 2. Find neighborhood around new_node
        neighbor_list = find_neighbors(new_node, rrt_dubins.node_list)
        
        # 2A. if neighborhood exists, find cheapest valid neighbor and connect
        if neighbor_list:
            cheapest_neighbor = find_cheapest_neighbor(new_node, neighbor_list, rrt_dubins)
            # if a cheapest neighbor exists, re-propogate and add new_node to tree
            if cheapest_neighbor:
                new_node = rrt_dubins.propogate(cheapest_neighbor, new_node)
                rrt_dubins.node_list.append(new_node)

        # 2B. if no neighborhood exists, find nearest node and connect if possible
        else:
            nearest_node = find_nearest_node(new_node, rrt_dubins.node_list)
            if not nearest_node:
                continue
            new_node = rrt_dubins.propogate(nearest_node, new_node)
            if not new_node:
                continue
            # check if path to nearest node has collisions
            if rrt_dubins.check_collision(new_node):
                rrt_dubins.node_list.append(new_node)           # Storing all valid nodes
            else:
                continue

        # 3. Rewire neighbors in neighbor list:
        if neighbor_list and cheapest_neighbor:
            for node in neighbor_list:
                if node.is_state_identical(cheapest_neighbor):
                    continue
                # check if cost through new_node is cheaper than current cost and rewire
                node = rewire_node(node, new_node, rrt_dubins)

        # Draw current view of the map
        # PRESS ESCAPE TO EXIT
        if display_map:
            rrt_dubins.draw_graph()       

        # 4. Backtrack from Goal Node to Start Node to Find Path 
        if new_node.is_state_identical(rrt_dubins.goal):
            goal = new_node
            # back calculate path starting at goal node and traversing each node's parent
            path_node_list = back_traverse_nodes(goal)
            return path_node_list

    if i==rrt_dubins.max_iter:
        print('reached max iterations: path not found')

    # if max iterations reached without finding a path, return None
    return None

def find_nearest_node(new_node, node_list):
    # helper function to search through ALL valid nodes and find closest node

    shortest_d = 50                         # initialize distance to max possible in map
    closest_node = None                     # initialize closest_node

    for node in node_list:                  # loop through every valid node (node_list)
        d = euclid_dist(node, new_node)     # find distance between node and new_node
        if d < shortest_d:                  # if dist is new low, store the node and dist
            closest_node = node
            shortest_d = d 
    return closest_node

def find_neighbors(new_node, node_list):
    # helper function to find all neighboring nodes within a defined radius
    n = len(node_list)                      # number of nodes (n)
    neighbor_node_list = []                 # intialize empty list to store neighbor nodes

    # calculate optimal radius
    if n > 10:
        r = 25*math.sqrt((math.log(n)/n))
    else:
        r = 15

    # loop through valid nodes and find nodes within radius of new_node
    for node in node_list:
        if node.is_state_identical(new_node):   # node cannot be neighbor of itself
            continue                        

        d = euclid_dist(node, new_node)     # find distance between new_node and valid node
        if d < r:                           # if node is within radius, add to neighborhood
            neighbor_node_list.append(node)

    return neighbor_node_list

def back_traverse_nodes(goal_node):
    """
    TASK: find shortest path from start_node to goal_node by traversing back from goal node
    through each parent node

    INPUT: goal_node       (node)     - starting point of the back-traversal is the goal node
    OUTPUT: path_node_list (list)     - list of nodes in order from start to goal
    """

    cur_node = goal_node                    # start at goal_node
    path_node_list = []                     # initialize empty list to store path nodes

    # traverse nodes until cur_node.parent = None (i.e. reached start node)
    while cur_node:                
        path_node_list.append(cur_node)     # append current node
        cur_node = cur_node.parent          # reset cur_node to its parent

    path_node_list.reverse()                # reverse list to start from goal_node

    return path_node_list

def euclid_dist(node_1, node_2):
    # helper function to calculate euclidean distance between two nodes
    
    dx = node_1.x - node_2.x                # delta x
    dy = node_1.y - node_2.y                # delta y
    return math.sqrt(dx**2 + dy**2)         # l2 norm between two nodes

def find_cheapest_neighbor(new_node, neighbor_node_list, rrt_dubins):
    """
    TASK: find the lowest cost-to-come by connecting new_node with each neighbor

    INPUT:
    new_node            (node)  - node to find cheapest neighbor for 
    neighbor_node_list  (list)  - list of all neighbors for new_node
    rrt_dubins          (obj)   - dubin_path_problem object to use propogate, collision check
                                  functions
    OUTPUT: 
    cheapest_neighbor   (node)  - neighbor that once connect to new_node has lowest cost
    """
    lowest_cost = 10000                     # initialize lowest cost
    cheapest_neighbor = None                # initialize cheapest_neighbor to node = None

    for node in neighbor_node_list:         # iterate through each neighbor
        new_node.cost = 0                   # reset new_node cost-to-come
        
        # update new_node with cost==course_length from node
        new_node = rrt_dubins.propogate(node, new_node)
        if not new_node:
            continue

        # if path from node to new_node is obstructed, skip to next node
        if not rrt_dubins.check_collision(new_node):
            continue

        # if new cost-to-come is lower, update cheapest node
        if new_node.cost < lowest_cost:
            cheapest_neighbor = node
            lowest_cost = new_node.cost
    
    # if all neighbors are obstructed, cheapest node will be None
    if not cheapest_neighbor:
        new_node.cost = 0
        return

    new_node.cost = 0                       # reset new_node

    return cheapest_neighbor

def rewire_node(node, new_node, rrt_dubins):
    # first check if node cost-to-come would be cheaper through new_node:
    cur_cost = node.cost        # current cost-to-come
    cur_node = rrt_dubins.propogate(new_node, node)     # get path and cost from new_node to cur_node

    if not rrt_dubins.check_collision(cur_node):        # no path exists so return original node
        return node
    
    if cur_node.cost < cur_cost:                        # if cost is better, rewire node through new_node
        node.parent = new_node
        node.cost = cur_node.cost
        node.path_x = cur_node.path_x
        node.path_y = cur_node.path_y
        node.path_yaw = cur_node.path_yaw

    return node