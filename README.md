abstract_planning_representation_net
====================================

Abstract Planning Representation Net learns an abstract representation for a high-dimensional robot motion planning problem which can be used to support classical planners (e.g., A*) through a highly informed heuristic or coarse-to-fine planning.

This repository contains the framework to extend a given high-dimensional planner by such an abstract representation. In detail, it contains methods to
- generate training data for the CNN
- train the CNN
- use the CNN as a planning heuristic


Please see the see the respective paper for further information:
"Learning Abstract Representations for Robot Locomotion Planning in High-dimensional Configuration Spaces" by Tobias Klamt and Sven Behnke.


Usage
-----

For the initial setup:
```
sudo apt-get install python-catkin-tools
cd planner_framework
source /opt/ros/kinetic/setup.bash
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-Wall
catkin build
echo ". $(pwd)/devel/setup.bash" >> ~/.bashrc

```

Generate Training Data
----------------------


