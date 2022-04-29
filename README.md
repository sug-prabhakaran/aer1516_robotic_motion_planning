# AER1516 - Motion Planning for Robotics

> This repository contains all course work for AER1516 - Motion Planning for Robotics course except for the final project, which can be found in the [aer1516_reliable_shortest_path_project](https://github.com/sug-prabhakaran/aer1516_reliable_shortest_path_project) repository.

* **Institution:** University of Toronto
* **Course:** AER1516 - Motion Planning for Robotics with Dr. Jonathan Kelly
* **Dates:** Winter 2022

## Assignment 1 - Motion Planning Theory

Assignment 1 was a written assignment that covered motion planning theory topics such as:

* proof of worst case distance for the Bug2 algorithm
* theory of singularity/degeneracy with Euler angles
* proof of covering charts using diffeomorphisms and manifold theory
* determining configuration space dimensions of various robot scenarios
* proof of workspace to configuration space mapping
* optimality of wavefront planner on a discrete grid

This assignment document was removed from this repo as per University of Toronto policy.

## Assignment 2 - Implementing RRT and RRT* Algorithms

In assignment 2, I implemented both the rapidly-expanding random tree (RRT) and RRT* algorithms for a robot complying with dubin's path kinematic constraints.  I then compared the performance of these two algorithms with the results shown in the PDF document.

![Example Output of RRT](/assignment2_RRT_and_RRT_star/img_rrt_on_iter_999.png)

![Example Output of RRT*](/assignment2_RRT_and_RRT_star/img_rrt_star_on_iter_9.png)

![Comparison of RRT and RRT*](/assignment2_RRT_and_RRT_star/img_comparison_rrt_rrt_star.png)

The code for this assignment was removed also as per University of Toronto policy.

## Paper Pres

## Packages Used

* python           - version 3.7.7
* numpy            - version 1.19.1
* matplotlib       - version 3.3.1

## References

1. **Gurobi Optimizer:** Gurobi is an industrial mixed-integer