#!/usr/bin/env python3
import rospy
from raiscar_msgs.msg import ControllerPath
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import json
import rospkg

path = rospkg.RosPack().get_path('freicar_control') + '/data/path.json'

with open(path) as f:
    poses = json.load(f)

pub = rospy.Publisher('freicar_1/path_segment', ControllerPath, queue_size=10)
pub_viz = rospy.Publisher('freicar_1/path_viz', PoseArray, queue_size=10)
rospy.init_node('exercise_path_publisher')

msg = ControllerPath()
viz_msg = PoseArray()

msg.des_vel = 0.2
viz_msg.header.stamp = msg.path_segment.header.stamp = rospy.Time.now()
viz_msg.header.frame_id = msg.path_segment.header.frame_id = 'map'

for p in poses:
    point = PoseStamped()
    viz_p = Pose()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = 'map'

    viz_p.position.x = point.pose.position.x = p[0]
    viz_p.position.y = point.pose.position.y = p[1]
    viz_p.position.z = point.pose.position.z = 0.0
    viz_p.orientation.w = point.pose.orientation.w = 1.0

    msg.path_segment.poses.append(point)
    viz_msg.poses.append(viz_p)

rospy.sleep(1)
pub.publish(msg)
print("subscribed and published")

while not rospy.is_shutdown():
    pub_viz.publish(viz_msg)
    rospy.Rate(10).sleep()


