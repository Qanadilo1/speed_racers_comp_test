#!/usr/bin/env python

from visualization_msgs.msg import MarkerArray
import rospy
from raiscar_msgs.msg import ControllerPath
from geometry_msgs.msg import PoseStamped, Pose, PoseArray

from visualization_msgs.msg import Marker

import pdb

import pdb
marker = Marker()
markerArray = MarkerArray()

markerArray.markers.append(marker)
markerArray.markers.append(marker)

pub = rospy.Publisher('freicar_1/path_segment', ControllerPath, queue_size=10)
pub_viz = rospy.Publisher('freicar_1/path_viz', PoseArray, queue_size=10)
rospy.init_node('control_planner')

flag = 0

# msg = ControllerPath()
# viz_msg = PoseArray()

# msg.des_vel = 0.2
# viz_msg.header.stamp = msg.path_segment.header.stamp = rospy.Time.now()
# viz_msg.header.frame_id = msg.path_segment.header.frame_id = 'map'
viz_msg = PoseArray()
viz_msg.header.stamp = rospy.Time.now()
viz_msg.header.frame_id  = 'map'

def callback(msg):
    # pdb.set_trace()
    control_msg = ControllerPath()
    control_msg.des_vel = 0.2
    control_msg.path_segment.header.stamp = rospy.Time.now()
    control_msg.path_segment.header.frame_id = 'map'

    viz_msg = PoseArray()
    viz_msg.header.stamp = rospy.Time.now()
    viz_msg.header.frame_id = 'map'

    poses = []
    for i in range(len(msg.markers)):
        poses.append([msg.markers[i].pose.position.x, msg.markers[i].pose.position.y])
        #print("poses",poses[i])


    for p in poses:
        point = PoseStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = 'map'
        control_msg.path_segment.poses.append(point)

        viz_p = Pose()

        viz_p.position.x = point.pose.position.x = p[0]
        viz_p.position.y = point.pose.position.y = p[1]
        viz_p.position.z = point.pose.position.z = 0.0
        viz_p.orientation.w = point.pose.orientation.w = 1.0

        viz_msg.poses.append(viz_p)

    pub_viz.publish(viz_msg)

    rospy.sleep(1)
    pub.publish(control_msg)


    #print("subscribed and published")




def subscriber_publisher():

    path_sub = rospy.Subscriber('planner_debug', MarkerArray, callback, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    subscriber_publisher()
