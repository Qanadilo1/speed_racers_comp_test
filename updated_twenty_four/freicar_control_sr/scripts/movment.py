import rospy
from raiscar_msgs.msg import ControlCommand
from freicar_common.msg import FreiCarControl


fctrl_msg = FreiCarControl()
def callback(data):
    global fctrl_msg
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.command)
    fctrl_msg.name = data.name
    fctrl_msg.command = data.command

pub = rospy.Publisher('/freicar_1/control', ControlCommand, queue_size=10)

mov_sub = rospy.Subscriber("/freicar_commands", FreiCarControl, callback, queue_size=10)


rospy.init_node('movment', anonymous= True)
rate = rospy.Rate(10)  # 10hz
while not rospy.is_shutdown():
    ctrl_msg = ControlCommand()
    ctrl_msg.steering = 0
    ctrl_msg.throttle = 0.1
    pub.publish(ctrl_msg)
    if fctrl_msg.command == "start":
        ctrl_msg = ControlCommand()
        ctrl_msg.steering = 0
        ctrl_msg.throttle = 0.3
        pub.publish(ctrl_msg)
    elif fctrl_msg.command == "stop":
        ctrl_msg = ControlCommand()
        ctrl_msg.steering = 0
        ctrl_msg.throttle = 0
        pub.publish(ctrl_msg)
    elif fctrl_msg.command == "straight":
        ctrl_msg = ControlCommand()
        ctrl_msg.steering = 0
        ctrl_msg.throttle = 0.1
        pub.publish(ctrl_msg)
    elif fctrl_msg.command == "left":
        ctrl_msg = ControlCommand()
        ctrl_msg.steering = 45
        ctrl_msg.throttle = 0.2
        pub.publish(ctrl_msg)
    elif fctrl_msg.command == "right":
        ctrl_msg = ControlCommand()
        ctrl_msg.steering = -45
        ctrl_msg.throttle = 0.1
        pub.publish(ctrl_msg)


