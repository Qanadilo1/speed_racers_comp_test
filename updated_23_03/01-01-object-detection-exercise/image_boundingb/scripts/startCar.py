import rospy
from raiscar_msgs.msg import ControlCommand
from sensor_msgs.msg import Image
import cv2
import numpy as np

# Author : Johan Vertens
# Descriptions: Sends a control command to the car to let it start driving straight

def img_callback(msg):
    np_img = np.fromstring(msg.data, dtype=np.uint8).reshape((720, 1280, 3))
    np_img = np_img[:, :, :3]
    bgr = np.zeros((np_img.shape[0], np_img.shape[1], 3), dtype=np.uint8)
    cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR, bgr, 3)
    cv2.imshow('RGB image', bgr)
    cv2.waitKey(10)  # This function is needed to display the image , it is blocking!

pub = rospy.Publisher('/freicar_1/control', ControlCommand, queue_size=10)
img_sub = rospy.Subscriber('/freicar_1/sim/camera/rgb/front/image', Image, callback=img_callback, queue_size=10)

rospy.init_node('startCar', anonymous=True)

rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    ctrl_msg = ControlCommand()
    ctrl_msg.steering = 0.5
    ctrl_msg.throttle = 0.1
    pub.publish(ctrl_msg)
    rate.sleep()