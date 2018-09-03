abstract_planning_representation_net
====================================

Abstract Planning Representation Net learns a 3-dimensional abstract representation for a high-dimensional robot motion planning problem which can be used to support classical planners (e.g., A*) through a highly informed heuristic or coarse-to-fine planning.

This repository contains the framework to extend a given high-dimensional planner by such an abstract representation. In detail, it contains methods to
- generate training data for the CNN,
- train the CNN, and
- use the CNN as a planning heuristic.

Please see the see the respective paper for further information:
"Learning Abstract Representations for Robot Locomotion Planning in High-dimensional Configuration Spaces" by Tobias Klamt and Sven Behnke.


Generate Training Data
----------------------
See the `src/training_data_generator.cpp` file to specifiy your planning problem:
- In the constructor, specify the parameters for your desired training data.
- According to your robot representation you need to specify the function `TransformAbstractPoseToDetailedPose`.

Start the necessary ros nodes:
```
roslaunch planning_abstraction_net planning_framework.launch
```
In another terminal, trigger the training data generation:
```
rosservice call /planning_abstraction_net/generate_training_data_from_random
```

Train the CNN
-------------
Specify the path to your training data in `scripts/abstraction_learning_training.py` and start the training:
```
python abstraction_learning_training.py
```

Use the CNN as a Planning Heuristic
-----------------------------------
Specify the name of your desired network checkpoint in the function `handle_load_network` in `abstraction_learning_ros_interface.py`.
Start the CNN ROS interface:
```
python abstraction_learning_ros_interface.py
```
In another terminal, trigger checkpoint loading:
```
rosservice call /abstraction_learning_network/load_network
```

In the `src/planner.cpp` make sure to call the `Init()` function as soon as the desired map is available. This triggers precomputation of the abstract representation and only needs to be done once per map. 

As soon as a planning goal is set, give this to the heuristic server with `m_heuristic_server.SetGoal()`. 
Now, the heuristic server provides you a heuristic to that goal for an arbitrary detailed pose in the map which you get with `m_heuristic_server.GetHeuristic()`.


License
-------
abstract_planning_representation_net is released under BSD-3.


Contact
-------
If you have any questions, mail Tobias Klamt (klamt@ais.uni-bonn.de).



