"""
Path planning Sample Code using RRT and RRT* with Dubins paths.
"""

import copy
import math
import random
import sys

import matplotlib.pyplot as plt
import numpy as np

try:
    import dubins_path_planning
    from rrt_planner import rrt_planner
    from rrt_star_planner import rrt_star_planner
except ImportError:
    raise

show_final_plot = True # TODO: true only in template

class RRT_dubins_problem():

    class Node():
        """
        RRT Node for dubins-type path

        Attributes
        ----------------
        x       - x coordinate of the node
        y       - y coordinate of the current node
        yaw     - yaw in radians of the current node
        parent  - a reference to the parent node object
                  (path goes from parent node to current node)
        cost    - The total langth of path to be travelled to reach
                  this node from the start location
        path_x  - a list of x coordiantes of all vehicle states included in the
                  dubins-type path joining parent node to the current node
        path_y  - a list of y coordiantes of all vehicle states included in the
                  dubins-type path joining parent node to the current node
        path_yaw- a list of yaw values coordiantes of all vehicle states included in the
                  dubins-type path joining parent node to the current node

        NOTE: x, y, yaw together constitute the state of the vehicle in the current node
        NOTE: path_x, path_y, path_y together constitute the complete path of the vehicle
              travelling from parent state to the current state
        """

        def __init__(self, x, y, yaw):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0
            self.yaw = yaw
            self.path_yaw = []

        def is_state_identical(self, node):
            """
                check if x, y, yaw are the identical
            """
            if abs(node.x - self.x) > 0.001:
                return False
            elif abs(node.y - self.y) > 0.001:
                return False
            elif abs(node.yaw - self.yaw) > 0.001:
                return False

            return True

        def print_node(self):
            """
                Node printing function. USE FOR DEBUGGING.
            """
            print(f'x: {self.x}, y:{self.y}, yaw: {self.yaw}')

    """
    RRT_DUBINS_PROBLEM Class for planning problems with Dubins-type paths.
    """

    def __init__(self, start, goal, obstacle_list, map_area,
                 max_iter=10000
                 ):
        """
        Init Parameter

        Inputs
        ------------------

        start         - Start Position (x,y)
        goal          - Goal Position [x,y]
        obstacle_list - Positions of circular obstacles [[x,y,size],...]
        map_area      - Random Sampling Area [min_x, max_x, min_y, max_y]
        max_iter      - Maximum number of random points that planning algorithm can create
                        (for avoiding infinite loops)
        """
        self.start = self.Node(start[0], start[1], start[2])
        self.goal = self.Node(goal[0], goal[1], goal[2])
        self.x_lim = map_area[0:2]
        self.y_lim = map_area[2:4]
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = [self.start] # List of all Valid nodes explored

        self.curvature = 1.0 # for dubins path

    def rrt_planning(self, display_map=False):
        """
            Calling function for your planning implementation
        """
        return rrt_planner(self, display_map=display_map)

    def rrt_star_planning(self, display_map=False):
        """
            Calling function for your planning implementation
        """
        return rrt_star_planner(self, display_map=display_map)

    def propogate(self, from_node, to_node):
        """
            PROPOGATE is a helper function that creates a new_node at the state
            specified by to_node and populates that node with a dubins-type path starting
            from state in from_node to state in to_node.

            Inputs
            --------------
            from_node - (Node) parent node of your path to be traversed
            to_node   - (Node) child node describing the goal state of this propogation

            Output
            --------------
            new_node - (Node) child node containing the state of to_node and the traversed path
        """
        px, py, pyaw, _, course_length = dubins_path_planning.dubins_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)

        if len(px) <= 1:  # cannot find a dubins path
            return None

        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += course_length
        new_node.parent = from_node

        return new_node

    def calc_dist_to_goal(self, x, y):
        """
            CALC_DIST_TO_GOAL is a helper function that calculates the
            distance of x and y to the goal state
        """
        dx = x - self.goal.x
        dy = y - self.goal.y
        return math.hypot(dx, dy)

    def calc_new_cost(self, from_node, to_node):
        """
            CALC_NEW_COST is a helper function that calculates the net path cost of going
            from from_node to to_node state.

            Inputs
            --------------
            from_node - (Node) parent node of the path to be traversed
            to_node   - (Node) child node describing the goal state of the total path

            Output
            --------------
            cost - (float) final path cost of a traversing from start to to_node
        """
        _, _, _, _, course_length = dubins_path_planning.dubins_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)

        return from_node.cost + course_length


    def check_collision(self, node):
        """
            CHECK_COLLISION checks if the path from parentNode to the current node collides
            with any of the obstacles

            Inputs
            ---------------
            node - (Node) Current Node

            Outputs
            ---------------
            (boolean) True for safe and False if collision occurs
        """
        if node is None:
            return False

        for (ox, oy, size) in self.obstacle_list:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size ** 2:
                return False  # collision

        return True  # safe

    def draw_graph(self):
        """
            DRAW_GRAPH, draws the problem setup, which includes:
            - all node
            - all paths between parent and chid nodes
            - all obstacles
            - start and goal point

            No Input or Output
        """
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
                plt.plot(node.x, node.y, "xb")   #ADDED BY ME TO VISUALIZE VALID NODES

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis([self.x_lim[0] - 2, self.x_lim[1] + 2, \
                    self.y_lim[0] - 2, self.y_lim[1] + 2])
        plt.grid(True)
        plt.pause(0.01) # Required for Macs
        self.plot_start_goal_arrow()
        plt.pause(0.01) # Required for Macs

    def plot_start_goal_arrow(self):
        """
            PLOT_START_GOAL_ARROW is a helper function used in DRAW_GRAPH function
        """
        dubins_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        dubins_path_planning.plot_arrow(
            self.goal.x, self.goal.y, self.goal.yaw)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

def check_path(rrt_problem, path_node_list : list):
    """
        CHECK_PATH This function checks if the output nodes list is
        a valid path which follows the follow conditions:
        1. node[0] is the start node, node[-1] is the goal node.
        2. node[i-1] is the parent of node[i].
        3. list of nodes represent a collision free path from start node to the goal node.
        4. node locations must be inside the map_area
    """
    # Implement this if needed
    return True

def get_path(path_node_list):
    """
        GET_PATH creates the x y path for display from the output nodes path
    """
    if not path_node_list:
        return []

    path = [[path_node_list[0].x, path_node_list[0].y]]
    for node in path_node_list[1:]:
        for (ix, iy) in zip(node.path_x, node.path_y):
            path.append([ix, iy])
        path.append([node.x, node.y])
    return path

def main():
    print("Executing: " + __file__)
    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = [0.0, 0.0, np.deg2rad(-50.0)]
    goal = [10.0, 10.0, np.deg2rad(50.0)]

    rrt_dubins = RRT_dubins_problem(start = start, goal = goal, \
                                    obstacle_list = obstacleList, \
                                    map_area = [-2.0, 15.0, -2.0, 15.0], \
                                    max_iter=200)
    path_node_list = rrt_dubins.rrt_star_planning(display_map=True)
    is_path_valid = check_path(rrt_dubins, path_node_list)
    path = get_path(path_node_list)

    if not path:
        print(f'Test Failed: Given path is empty\n Visualize the path to debug')
        return
    if not is_path_valid:
        print(f'Test Failed: Given path is not valid\n Visualize the path to debug')
        return

    print("start:", start, "goal:", goal)
    print("bounds:", rrt_dubins.x_lim, rrt_dubins.y_lim)

    # Draw final path
    if show_final_plot:
        rrt_dubins.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.001) # Necessary for macs
        plt.show()

if __name__ == '__main__':
    main()
