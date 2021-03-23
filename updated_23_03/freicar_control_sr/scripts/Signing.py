import rospy
from raiscar_msgs.msg import ControlCommand
from freicar_common.msg import FreiCarControl
from freicar_common.msg import FreiCarSigns

fctrl_msg = FreiCarSigns()
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
    fctrl_msg.signs = data.signs

pub = rospy.Publisher('/freicar_1/control', ControlCommand, queue_size=10)

mov_sub = rospy.Subscriber("/traffic_signs", FreiCarSigns, callback, queue_size=10)


rospy.init_node('movment', anonymous= True)
rate = rospy.Rate(10)  # 10hz
while not rospy.is_shutdown():
    if(len(fctrl_msg.signs)!=0):
        print(fctrl_msg.signs[0].type)
    ctrl_msg = ControlCommand()
    ctrl_msg.steering = 0
    ctrl_msg.throttle = 0
    if (len(fctrl_msg.signs) != 0):
        if(fctrl_msg.signs[0].type == 1):
            ctrl_msg.throttle = 0
            pub.publish(ctrl_msg)
            rate = rospy.Rate(1)
    pub.publish(ctrl_msg)

