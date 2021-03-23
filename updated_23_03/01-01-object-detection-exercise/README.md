# Explanation on how to run the ROS Node

We created a new package
 **_image_boundingb_** that defines a node that will subscribe to the image topic and publish the bounding boxes.

**_image_sub_pub.py_** is the script where the publisher and subscriber are defined.

First the catkin workspace needs to be built to add the new package 
```console
cd freicar_ws
catkin build
```

# Launching the simulator 
As shown in the FreiCAR2020 slides, launch this to get the image from the front camera published to a rostopic
```console
roslaunch freicar_launch sim_base.launch
```

# Running the ROS Node

The node with the subscriber/publisher node is located outside the package

```console
cd 01-01-object-detection-exercise
python image_sub_pub.py
```
After running this script, a new rostopic will appear and can be echoed over using
```console
rostopic echo /freicar_1/bounding_box
rqt_graph
```
The ROS rqt_graph will show the following publisher/subscriber nodes

![alt text](https://github.com/NayaBaslan/speed_racers_group_submissions/blob/main/01-01-object-detection-exercise/rosgraph.png)


# Running the evaluation code

We included only one file in the logs folder from the training
```console
python evaluate.py -w logs/freicar-detection/efficientdet-d0_95_166000.pth
```
