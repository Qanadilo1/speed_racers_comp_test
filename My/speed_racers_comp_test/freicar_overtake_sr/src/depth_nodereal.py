#!/usr/bin/env python
import rospy


from sensor_msgs.msg import Image
from bounding_box import bounding_box as bb
from raiscar_msgs.msg import ControlCommand
from freicar_bb_box_sr.msg import bb as bb_msg
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
    # pdb.set_trace()
    return np.mean(arr[k+1:n-k])


def box_callback(msg):
    global x1, x2, y1, y2

    # # print(msg.x1)
    # x1 = int(msg.x1)
    # x2 = int(msg.x1+msg.x2)
    # y1 = int(msg.y1)
    # y2 = int(msg.y1+msg.y2)

    x1 = int(msg.x1)
    x2 = int(msg.x2)
    y1 = int(msg.y1)
    y2 = int( msg.y2)
    # print("bounding boxes  = ", msg.x1, msg.x2, msg.y1, msg.y2)

    # print(x1, x2, y1 ,y2)
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

    bb_center = ( x1 + y1 ) / 2
    image_center = np.shape(image2)[0] / 2
    angle_car = (bb_center - image_center) / image_center * (fov/2)
    depth_f_min = np.min(depth_f)

    print("angle" , angle_car)
    print("depth", depth_f_min)

    #############################
    # image = cv2.copyMakeBorder(image, top=12, bottom=12, left=0,
    #                           right=0, borderType=cv2.BORDER_CONSTANT, value=[0, 255, 0])
    # depth_sign = image.min()
    # depth = image[x1:x2+x1, y1:y2+y1].min()
    # depth_sr = image[x1:x2+x1, y1:y2+y1]

    # depth = image[x1:x2 , y1:y2 ].min()
    # depth_sr = image[x1:x2, y1:y2 ]
    # pdb.set_trace()

    # mean_trim = trimmean(np.reshape(depth_sr,(-1,1)), 0.9)
    # print("mean = " , mean_trim , "min", depth)
    # for i in range(np.shape(depth_sr)[1]):
    #     print(depth_sr[0:(np.shape(depth_sr)[0]-1)][i])


    # depth_sr   MATRIX WITH DEPTH VALUES INSIDE BB

    # height = np.shape(depth_sr)[1]
    # width = np.shape(depth_sr)[0]
    #
    # middle_point_width_low = int(width/2) - 30
    # middle_point_width_high = int(width / 2) + 30
    #
    # middle_point_height_low = int(height / 3) - 15
    # middle_point_height_high = int(height / 3) + 15

    # depth_sr_car = depth_sr[middle_point_width_low:middle_point_width_high, middle_point_height_low:middle_point_height_high]
    # print("average = ", np.average(depth_sr_car))

    # pdb.set_trace()



    # middle_point = depth_sr[int(width/2)][int(height/3)]
    # print("middle point", middle_point)
    # test = np.reshape(image[x1:x2, y1:y2], (-1, 1))
    # for i in range(np.shape(test)[0]):
    #     print(test[i])
    # pdb.set_trace()
    # print(depth)

    # cv2.imshow("Image", image)
    #cv2.waitKey(3)
    # pdb.set_trace()


    ###
    # original image = [1280 , 720]
    # model image    = [384  , 640]
    # bb image       = [384  , 640]

    overtake_publisher = rospy.Publisher('/overtake', Bool, queue_size=10)
    # collision_publisher = rospy.Publisher('min_depth', Float64, queue_size=10)

    image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    # bb.add(image, x1, y1, x2, y2, color="green")
    cv2.rectangle(image, (x1, x2), (y1, y2), (0, 255, 0))
    # cv2.imshow("Image", image)
    # cv2.waitKey(1)
    # print(depth)
    print("depth is " ,depth_f_min)
    if (depth_f_min < 0.45):
        # print("im less than 0.65")
        overtake = True

        overtake_publisher.publish(overtake)
        time.sleep(1.5)

        overtake = False
        overtake_publisher.publish(overtake)
        time.sleep(9)


    # min_depth_dist = depth_sign
    # collision_publisher.publish(min_depth_dist)
def depth_node():

# Init pusblisher and subscriber
    rospy.init_node('Seg_Reg_node', anonymous=True)
    bb_box = rospy.Subscriber('/freicar_1/bounding_box',
                             bb_msg, callback=box_callback, queue_size=10)
    # pdb.set_trace()
    rospy.Subscriber('/freicar_1/sim/camera/depth/front/image_float', Image, callback=img_callback, queue_size=1)



    # init rosnode
    #rospy.init_node('Seg_Reg_node', anonymous=True)
    rospy.spin()

if __name__ == '__main__':


    depth_node()
'''
rate = rospy.Rate(0.01)  # 10hz
while not rospy.is_shutdown():
    rate.sleep()
'''
