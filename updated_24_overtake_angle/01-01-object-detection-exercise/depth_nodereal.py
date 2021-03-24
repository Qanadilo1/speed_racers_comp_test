import rospy


from sensor_msgs.msg import Image
from bounding_box import bounding_box as bb
from raiscar_msgs.msg import ControlCommand
from image_boundingb.msg import bb as bb_msg
from std_msgs.msg import Bool

import pdb
import cv2
from std_msgs.msg import Float64
import numpy as np

from cv_bridge import CvBridge
import time

fov = 84.8711
def trimmean(arr, percent):
    n = len(arr)
    k = int(round(n*(float(percent)/100)/2))
    return np.mean(arr[k+1:n-k])


def box_callback(msg):
    global x1, x2, y1, y2

    x1 = int(msg.x1)
    x2 = int(msg.x2)
    y1 = int(msg.y1)
    y2 = int( msg.y2)

# Callback function called for every image received

def img_callback(msg):
    overtake = True;
    global bridge
    #print(msg)
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, "passthrough")
    # pdb.set_trace()
    # image = cv2.resize(image, (384,640))
    image = cv2.resize(image, (640, 384))




    ###############################
    image2 = cv2.resize(image, ( 384, 640))
    depth_f = image2[x1:y1, x2:y2]

    bb_center = (x1 + y1) / 2
    image_center = np.shape(image2)[0] / 2
    angle_car = (bb_center - image_center) / image_center * (fov/2)
    depth_f_min = np.min(depth_f)
    global theta
    theta = angle_car
    print("angle" , angle_car)
    print("depth", depth_f_min)
    if (theta < 4.0 and theta > -4.0):
        theta_overtake = True
    else:
        theta_overtake = False


    overtake_publisher = rospy.Publisher('/overtake', Bool, queue_size=10)

    image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    cv2.rectangle(image, (x1, x2), (y1, y2), (0, 255, 0))
    # cv2.imshow("Image", image)
    # cv2.waitKey(1)

    print("depth is " ,depth_f_min)
    if (depth_f_min < 0.45 and theta_overtake ):
        overtake = True

        overtake_publisher.publish(overtake)
        time.sleep(2)

        overtake = False
        overtake_publisher.publish(overtake)
        time.sleep(9)

def depth_node():

# Init pusblisher and subscriber
    rospy.init_node('Seg_Reg_node', anonymous=True)
    bb_box = rospy.Subscriber('/freicar_1/bounding_box',
                             bb_msg, callback=box_callback, queue_size=10)

    rospy.Subscriber('/freicar_1/sim/camera/depth/front/image_float', Image, callback=img_callback, queue_size=1)

    rospy.spin()

if __name__ == '__main__':


    depth_node()
'''
rate = rospy.Rate(0.01)  # 10hz
while not rospy.is_shutdown():
    rate.sleep()
'''
