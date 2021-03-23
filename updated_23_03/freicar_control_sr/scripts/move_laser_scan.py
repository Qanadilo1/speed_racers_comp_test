#!/usr/bin/env python

import rospy
import math
import pdb
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
'''from sensor_msgs.msg import Float32'''
import numpy as np
# form start car
from raiscar_msgs.msg import ControlCommand
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import sys, select, termios, tty
range_center = Float32()
range_left = Float32()
range_right = Float32()
range_left_last = Float32()
range_right_last = Float32()

normal_vel =0.2
normal_angel_turn =1.57284
last_ob_vel =0
last_ob_angel =0.21845
half_angel =1.57284

data = PointCloud2

data.
def callback(data):
    # print(data.header)
    # pdb.set_trace()
    counter = 0
    index = 0

    print("shape ", data.width)
    list_z = []
    list_y = []
    list_x = []
    total_x = 0
    listx_z = []
    listy_z = []

    tot_x = 0

    indices_crash = []
    for p in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        total_x = total_x + p[0]
        # list_z.append(p[2])
        # list_x.append(p[0])
        # list_y.append(p[1])
        # if (p[2] > 0.5):
        list_z.append(p[2])
        list_x.append(p[0])
        list_y.append(p[1])


    for i in range(len(list_x)):
        if (list_x[i] >0.2) :
            listx_z.append(list_x[i])
            listy_z.append(list_y[i])

            tot_x = tot_x + list_x[i]

    # for i in range(len(list_x)):
    #     if(list_x[i] < 2 and list_x[i] > 0.5 ):
    #         print("index", i)

    # normalize y values to see exactly in the center
    middle_y = np.abs(np.max(list_y) + np.min(list_y))/2
    pdb.set_trace()
    #
    # if (len(listx_z) > 0):

        # for i in range(len(listx_z)-1):
        #     if (listx_z[i] - listx_z[i+1] > 10 and listx_z[i+1]<1.0):
        #         # print("far away sign sign found at distance = ", listx_z[i+1])
        #         print("danger, need to steer left")
        #         # if (listx_z[i] - listx_z[i+1] > 10 and listx_z[i+1]>1)
        #         # pdb.set_trace()
        #         break

        # print(np.min(listx_z))
        # print("avg x", tot_x/len(listx_z))
        # print(listx_z)
    # for i in range(len(listx_z)):
    #     total_x = total_x +
    # pdb.set_trace()
    # print(total_x/len(list_x))
    # pdb.set_trace()

        # print("index",index, "px = " ,p[0], "py = " ,p[1], "pz = " ,p[2], )

    #     # pdb.set_trace()
    #     list_z.append(p[2])
    #     if (p[2] > 2.1):
    #         counter = counter +1
    #         indices_crash.append(p[0])
    #
    #     index = index + 1
    # # pdb.set_trace()
    # # print("total points > 2.1 = " , counter)
    # max_list =np.max( list_z)
    # # print("max in list", max_list)
    # tot_x = 0
    # if(counter > 15):
    #     # print("i will crash")
    #     for x  in indices_crash:
    #         # print("yes")
    #         # print("x  of crash", x )
    #         tot_x = tot_x + x
    #         # pdb.set_trace()
    #     avg_x = tot_x/counter
    #     print("avg x = ", avg_x)
    #
    #
    #     if (avg_x > 2 and avg_x < 14):
    #         print("danger steer to left")
    # # else:
        # print("safe")
    # pdb.set_trace()


        # if (p[2] > 1.9):
        #     print("steer to the left")
        # else:
        #     print("no need to steer left")

    # pdb.set_trace()
        # print(" x : %f  y: %f  z: %f" % (p[0], p[1], p[2])


    # rospy.loginfo("ranges %f", data.ranges[359])
    # range_center.data= data.ranges[359]
    # range_left.data= data.ranges[180]
    # range_right.data= data.ranges[540]
    # range_left_last.data= data.ranges[20]
    # range_right_last.data= data.ranges[700]
    # print(data)

def move():
    '''turtlebot_teleopr'''
    pub_ls = rospy.Publisher('/freicar_1/control', ControlCommand, queue_size=10)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    sub = rospy.Subscriber("freicar_1/sim/lidar", PointCloud2, callback)
    rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    twist = Twist()
    while not rospy.is_shutdown():

        # if (range_center.data>1):
        #         twist.linear.x =normal_vel; twist.linear.y = 0; twist.linear.z = 0
        #         twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        #         pub.publish(twist)
        # if (range_left_last.data<0.2):
        #         twist.linear.x = last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
        #         twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = last_ob_angel
        #         pub.publish(twist)
        #
        # elif (range_right_last.data<0.2):
        #         twist.linear.x = last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
        #         twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -last_ob_angel
        #         pub.publish(twist)
        #         print(range_center.data)
        #
        # else :
		# 	if (range_left_last.data>1) or (range_left.data>1):
		# 		twist.linear.x = last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
		# 		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -normal_angel_turn
		# 		pub.publish(twist)
        #
        #
		# 	elif (range_right_last.data>1) or (range_right.data>1) :
		# 		twist.linear.x = last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
		# 		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = normal_angel_turn
		# 		pub.publish(twist)
        #
		#     	elif (range_left.data>range_right.data):
		# 	    	twist.linear.x = last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
		# 	    	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -normal_angel_turn
		# 	    	pub.publish(twist)
        #
		#     	else :
		# 	    	twist.linear.x =last_ob_vel; twist.linear.y = 0; twist.linear.z = 0
		# 	   	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = normal_angel_turn
		# 	    	pub.publish(twist)

        # print("subscribed to laser")


                   
        rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass


