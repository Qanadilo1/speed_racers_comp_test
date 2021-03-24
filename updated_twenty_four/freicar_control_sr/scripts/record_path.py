#!/usr/bin/env python
import roslib
import rospy
import tf
import numpy as np
import json

dest = '/home/vertensj/freicar_ws/src/raiscar_ros/freicar_control/data/path.json'

if __name__ == '__main__':
    rospy.init_node('path_recorder')

    listener = tf.TransformListener()

    last_t = np.array([0, 0])
    rate = rospy.Rate(100.0)

    poses  = []
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/freicar_1/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        current_t = np.array([trans[0], trans[1]])

        dist = np.linalg.norm(last_t - current_t)

        if dist > 0.1:
            poses.append([current_t[0], current_t[1]])
            last_t = current_t
            print('added point')

        rate.sleep()

    print('Saving path...')

    with open(dest, 'w') as json_file:
        json.dump(poses, json_file)



