"""
Assignment #2 Template file
"""
import random
import math
import numpy as np

"""
Problem Statement
--------------------
Implement the planning algorithm called Rapidly-Exploring Random Trees (RRT)
for the problem setup given by the RRT_DUBINS_PROMLEM class.

INSTRUCTIONS
--------------------
1.  The only file to be submitted is this file rrt_planner.py. Your implementation
    can be tested by running RRT_DUBINS_PROBLEM.PY (check the main function).
2.  Read all class and function documentation in RRT_DUBINS_PROBLEM carefully.
    There are plenty of helper function in the class to ease implementation.
3.  Your solution must meet all the conditions specificed below.
4.  Below are some do's and don'ts for this problem as well.

Conditions
-------------------
There are some conditions to be satisfied for an acceptable solution.
These may or may not be verified by the marking script.

1.  The solution loop must not run for more that a certain number of random iterations
    (Specified by a class member called MAX_ITER). This is mainly a safety
    measure to avoid time-out-related issues and will be set generously.
2.  The planning function must return a list of nodes that represent a collision-free path
    from start node to the goal node. The path states (path_x, path_y, path_yaw)
    specified by each node must define a Dubins-style path and traverse from node i-1 -> node i.
    (READ the documentation for the node class to understand the terminology)
3.  The returned path should have the start node at index 0 and goal node at index -1,
    while the parent node for node i from the list should be node i-1 from the list, ie,
    the path should be a valid list of nodes.
    (READ the documentation of the node to understand the terminology)
4.  The node locations must not lie outside the map boundaries specified by
    RRT_DUBINS_PROBLEM.map_area.

DO(s) and DONT(s)
-------------------
1.  Do not rename the file rrt_planner.py for submission.
2.  Do not change change the PLANNING function signature.
3.  Do not import anything other than what is already imported in this file.
4.  You can write more function in this file in order to reduce code repitition
    but these function can only be used inside the PLANNING function.
    (since only the planning function will be imported)
"""

def rrt_planner(rrt_dubins, display_map=False):
    """
        Execute RRT planning using Dubins-style paths. Make sure to populate the node_list.

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
    i = 0
    while i < rrt_dubins.max_iter:
        i += 1
        print("\niter #:", i)
        # Generate a random vehicle state (x, y, yaw)
        position = np.around(17*np.random.rand(2)-2,1)            # random x, y between (-2, 15)
        heading = np.deg2rad(np.around(360*np.random.rand(1),1))  # random theta between (0, 360)
        rand_state = np.concatenate([position, heading])
        #print("rand_state:", rand_state)
        new_node = rrt_dubins.Node(rand_state[0], rand_state[1], rand_state[2])

        # Find an existing node nearest to the random vehicle state
        print("\nnew_node:")
        new_node.print_node()
        nearest_node = find_nearest_node(new_node, rrt_dubins.node_list)
        #print("nearest_node:")
        #nearest_node.print_node()
        new_node_prop = rrt_dubins.propogate(nearest_node, new_node)
        print("new_node, parent:")
        new_node_prop.print_node(), new_node_prop.parent.print_node()
    
        print("\nNodes from node_list:")
        for j, node in enumerate(rrt_dubins.node_list):
            print(j,": "), node.print_node()

        # Check if the path between nearest node and random state has obstacle collision
        # Add the node to nodes_list if it is valid
        if rrt_dubins.check_collision(new_node_prop):
            rrt_dubins.node_list.append(new_node_prop) # Storing all valid nodes
        else:
            continue

        # Draw current view of the map
        # PRESS ESCAPE TO EXIT
        if display_map:
            rrt_dubins.draw_graph()

        # Check if new_node is close to goal
        if rrt_dubins.calc_dist_to_goal(new_node.x, new_node.y) < 1:
            print("Iters:", i, ", number of nodes:", len(rrt_dubins.node_list))
            rrt_dubins.node_list.append(rrt_dubins.goal)
            break

        if i > 25:
            break

    if i == rrt_dubins.max_iter:
        print('reached max iterations')

    # Return path, which is a list of nodes leading to the goal...
    return None

def find_nearest_node(new_node, node_list):
    shortest_d = 21

    for node in node_list:
        d = ((new_node.x - node.x)**2 + (new_node.y - node.y)**2)**(1/2)
        if d < shortest_d:
            closest_node = node
            shortest_d = d 
    return closest_node