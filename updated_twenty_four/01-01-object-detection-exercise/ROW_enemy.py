import rospy


from sensor_msgs.msg import Image
from bounding_box import bounding_box as bb
from raiscar_msgs.msg import ControlCommand
from image_boundingb.msg import bb as bb_msg
from std_msgs.msg import Bool
from std_msgs.msg import Float64

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import pdb
import cv2

import numpy as np

from cv_bridge import CvBridge



import tf2_ros
import geometry_msgs.msg


fov = 84.8711

def box_callback(msg):
    global x1, x2, y1, y2
    global check_bb

    x1 = int(msg.x1)
    x2 = int(msg.x2)
    y1 = int(msg.y1)
    y2 = int( msg.y2)

    if (x1 == 0 and x2 == 0 and y1 ==0 and y2 ==0):  #Overall
        check_bb = 0
    else:
        check_bb = 1
    rospy.Subscriber('/freicar_1/sim/camera/depth/front/image_float', Image, callback=img_callback, queue_size=1)



def img_callback(msg):
    overtake = False;
    global bridge
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, "passthrough")
    image = cv2.resize(image, (640, 384))
    image2 = cv2.resize(image, ( 384, 640))

    depth_f = image2[x1:y1, x2:y2]

    bb_center = ( x1 + y1 ) / 2
    image_center = np.shape(image2)[0] / 2
    angle_car = (bb_center - image_center) / image_center * (fov/2)
    global theta
    theta = angle_car
    # print("theta inimg", theta)

    # print("angle" , angle_car)
    # print("depth", np.min(depth_f))

    image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    cv2.rectangle(image, (x1, x2), (y1, y2), (0, 255, 0))

    # pdb.set_trace()
    cv2.imshow("Image", image)
    cv2.waitKey(1)

    overtake_publisher = rospy.Publisher('/overtake', Bool, queue_size=10)
    overtake_publisher.publish(overtake)

    # print("theta before callback", theta)

    sub = rospy.Subscriber("freicar_1/sim/lidar", PointCloud2, callback_lidar)



def callback_lidar(data):
    # print("are we ever here")
    counter = 0
    index = 0
    list_z = []
    list_y = []
    list_x = []
    total_x = 0
    global car_dist

    for p in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        list_z.append(p[2])
        list_x.append(p[0])
        list_y.append(p[1])

    # pdb.set_trace()

    if(check_bb == 1):
        print("the bb was found")
        for i in range(len(list_x) - 1):

            # print(list_x)
            # pdb.set_trace()
            if (list_x[i] - list_x[i + 1] > 5):
                # if (abs(list_x[i + 1] - list_x[i + 2]) < 0.1 and abs(list_x[i + 2] - list_x[i + 3]) < 0.1 and abs(
                #         list_x[i + 3] - list_x[i + 4]) < 0.1
                #         and abs(list_x[i + 4] - list_x[i + 5]) < 0.1 and abs(list_x[i + 5] - list_x[i + 6]) < 0.1 and abs(
                #             list_x[i + 6] - list_x[i + 7]) < 0.1
                #         and abs(list_x[i + 7] - list_x[i + 8]) < 0.1 and abs(list_x[i + 8] - list_x[i + 9]) < 0.1):


                if (abs(list_x[i + 1] - list_x[i + 2]) < 0.1 and abs(list_x[i + 2] - list_x[i + 3]) < 0.1
                        and abs(list_x[i + 3] - list_x[i + 4]) < 0.1 and abs(list_x[i + 4] - list_x[i + 5]) < 0.1
                        and abs(list_x[i + 4] - list_x[i + 5]) < 0.1 and abs(list_x[i + 5] - list_x[i + 6]) < 0.1 ):

                    print("value", list_x[i + 1])

                    obs_x = list_x[i + 1]
                    obs_y = 1.7 * np.tan(theta)
                    car_dist = list_x[i + 1]     #Overall

                    # pdb.set_trace()

                    br = tf2_ros.TransformBroadcaster()
                    t = geometry_msgs.msg.TransformStamped()
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = "freicar_1/base_link"
                    t.child_frame_id = "enemy_agent"
                    t.transform.translation.x = list_x[i+1] + 0.512
                    # print("lidar distance = " , list_x[i+1])
                    t.transform.translation.y = list_x[i+1] * np.tan(theta*0.0174)
                    t.transform.translation.z = 0.0
                    quat = quaternion_from_euler(0, 0, theta)



                    # pdb.set_trace()
                    t.transform.rotation.x = quat[0]
                    t.transform.rotation.y = quat[1]
                    t.transform.rotation.z = quat[2]
                    t.transform.rotation.w = quat[3]

                    br.sendTransform(t)
                    break

    # Publisher that the controller subscribes to to see that it should stop
    bb_detect_pub = rospy.Publisher('bb_detect', Bool, queue_size=10)
    bb_detect_pub.publish(check_bb)
    dist_detect_pub = rospy.Publisher('dist_detect', Float64, queue_size=10)
    dist_detect_pub.publish(car_dist)
    angle_detect_pub = rospy.Publisher('angle_detect', Float64, queue_size=10)
    angle_detect_pub.publish(theta)

def depth_node():

    rospy.init_node('row_enemy', anonymous=True)
    bb_box = rospy.Subscriber('/freicar_1/bounding_box',
                             bb_msg, callback=box_callback, queue_size=10)


    rospy.spin()

if __name__ == '__main__':


    depth_node()



