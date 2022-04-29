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

In assignment 2, I implemented both the rapidly-expanding random tree (RRT) and RRT* algorithms for a robot complying with dubin's path kinematic constraints.  I then compared the performance of these two algorithms with the results shown in the PDF document and below in Figure 3.

Figure 1 and 2 depict an example of a particular random instance of each RRT and RRT* algorithm respectively using Dubin's path constraints.

<p align="center">
  <b>Figure 1 - Example Output of RRT Algorithm with Dubin's Path <br></b>
  <img src="/assignment2_RRT_and_RRT_star/img_rrt_on_iter_999.png" width="350" title="Example Output of RRT Algorithm with Dubin's Path">
</p>

<p align="center">
  <b>Figure 2 - Example Output of RRT* Algorithm with Dubin's Path <br></b>
  <img src="/assignment2_RRT_and_RRT_star/img_rrt_star_on_iter_9.png" width="350" title="Example Output of RRT* Algorithm with Dubin's Path">
</p>
<p align="center">
  <b>Figure 3 - Comparison of Performance of RRT vs RRT* Algorithms <br></b>
  <img src="/assignment2_RRT_and_RRT_star/img_comparison_rrt_rrt_star.png" width="350" title="Comparison of Performance of Both Algorithms">
</p>

The code for this assignment was removed also as per University of Toronto policy.

## Paper Presentation

The paper presentation was on the paper by Nardi \[1\] on path planning in an uncertain environment.  The attached jupyter notebook was used to outline the main steps of the algorithm outlined in the paper and to produce graphics to visualize the uncertainty and posterior of the position of the robot in a discrete grid.  Please see the jupyter notebook for more details.


## Packages Used

* python           - version 3.7.7
* numpy            - version 1.19.1
* matplotlib       - version 3.3.1
* scipy            - version 2.18.2

## References

\[1\] L. Nardi and C. Stachniss, "Uncertainty-Aware Path Planning for Navigation on Road Networks Using Augmented MDPS," in 2019 International Conference on Robotics and Automation (ICRA), May 20-24, 2019, Montreal, Canada \[Online\]. Available: IEEE Xplore, https://ieeexplore.ieee.org/document/8794121. [Accessed: 01 March 2022].

\[2\] O. Vysotska and C. Stachniss, "Improving SLAM by Exploiting Building Information from Publicly Available Maps and Localization Priors," PFG- Journal of Photogrammetry, Remote Sensing and Geoinformation Science, vol. 85, pp. 53-65, 21 February 2017. \[Online\]. Available: Springer Link, https://link.springer.com/article/10.1007/s41064-017-0006-3. [Accessed: 08 March 2022].