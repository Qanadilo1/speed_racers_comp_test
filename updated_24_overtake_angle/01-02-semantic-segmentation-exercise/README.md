# Explanation on how to run the ROS Node

# ROS Part 1: Exercise Task 2.5.1 - Publish Segmentation and Lane Regression

**_seg_reg_publihser.py_** is the script where the classes and lanes publisher and front camera image subscriber are defined.

After lauching the rosnodes with the simulator, in a new terminal run this python script
```console
roslaunch freicar_launch sim_base.launch
cd 01-02-semantic-segmentation-exercise
python seg_reg_publihser.py
```

# ROS Part 2: Exercise Task 2.5.2 - Publish lane centers as MarkerArray and visualize in rviz
```console
cd 01-02-semantic-segmentation-exercise
python seg_reg_publihser.py
```

We saved our rviz configuration and pushed it into this repo for easier use
```console
cd 01-02-semantic-segmentation-exercise
rviz view_images_and_lanecenters
```

Screen Capture from rviz showing the original image from the camera, the lane regression image, and the classes segmentation image and the lane center points as MarkerArray

![alt text](https://github.com/NayaBaslan/speed_racers_group_submissions/blob/main/01-02-semantic-segmentation-exercise/cla_reg_lanes.png)
