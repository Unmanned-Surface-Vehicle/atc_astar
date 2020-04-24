# ATC A* - CPP

A COLREGs-Compliant USV Local Planner ROS plugin.
A* with Artificial Terrain Cost for Search Space Restriction.
ROS plugin currently being used together with USV_sim USV simulator as local planner.

## ATC A*
@brief: Implementation of a modification of the method presented by Agrawal et al., for COLREGS-compliance through the creation of virtual obstacles.

Agrawal,  P.;  Dolan,  J.  M.  “COLREGS-compliant  target  following  for  an  UnmannedSurface Vehicle in dynamic environments”,IEEE International Conference on IntelligentRobots and Systems, vol. 2015-Decem, 2015, pp. 1065–1070.

## How to Install

    $ cd catkin_ws/src
    $ git clone https://github.com/Unmanned-Surface-Vehicle/atc_astar.git
    $ cd ~/catkin_ws
    $ catkin_make_isolated --install --pkg atc_astar
    $ source install_isolated/setup.bash

## How to Run

The atc_astar plugin can be executed after the execution of the USV_sim:

    $ roslaunch usv_sim movebase_navigation1.launch parse:=true
    $ roslaunch usv_sim movebase_navigation1.launch parse:=false  


# ATC A*


## Improvements

1. A* heuristic

1.1. Proportion between obstacles cost and distance

1.2. Reduce turns. Smoother path

