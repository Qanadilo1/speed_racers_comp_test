
#Exercise 4, Control, Pure Pursuit Controller
## How to start the node:
Run `roslaunch freicar_control start_controller.launch`.

## How to send a path:
Run `rosrun freicar_control pub_path.py`. This node will send a predefined path to the controller and additionally publishes it as PoseArray message that can be visualized using Rviz.
 
## What you need to implement:
You need to implement the Pure Pursuit Controller in `controller_step` within `pure_pursuit.cpp`.
The function is called everytime there is a new odometry incoming.
The path to be followed is automatically received and is accessible by reading the `path_` member variable (defined in `controller.h` )

Note that for this exercise it is allowed to use GT positions (comp_mode = false). However in the competition you need to use your localizer.

Please note all comments before the respective function definitions.

Author: Johan Vertens (vertensj@informatik.uni-freiburg.de)
