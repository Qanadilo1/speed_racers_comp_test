# Exercise 3, Localization, Particle Filter
## How to start the node:
We created a new rosnode to publish the birds eye view for the lane regression. To run it:

`python bev_lanes_img.py`

To use lane regression, in the `evaluate.launch` set the parameter to

`<arg name="use_lane_regression" default="true" />`

and run the above mentioned rosnode.

To run without lane regression, use:

`<arg name="use_lane_regression" default="false" />`