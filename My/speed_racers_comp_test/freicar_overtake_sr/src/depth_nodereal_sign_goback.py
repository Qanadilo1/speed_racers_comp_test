#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float64

import cv2

from cv_bridge import CvBridge


def img_callback(msg):

    global bridge

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, "passthrough")
    image = cv2.resize(image, (640, 384))

    depth = image.min()
    cv2.imshow("Image", image)
    cv2.waitKey(3)

    min_depth_dist = depth

    print("Image",image.min())

    collision_publisher = rospy.Publisher('min_depth', Float64, queue_size=10)
    collision_publisher.publish(min_depth_dist)




def depth_node_stop():

    rospy.init_node('stop_collision_with_sign', anonymous=True)

    rospy.Subscriber('/freicar_1/sim/camera/depth/front/image_float', Image, callback=img_callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':


    depth_node_stop()
'''
rate = rospy.Rate(0.01)  # 10hz
while not rospy.is_shutdown():
    rate.sleep()
'''
