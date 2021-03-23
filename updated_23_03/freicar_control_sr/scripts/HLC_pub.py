import rospy
from freicar_common.msg import FreiCarControl


rospy.init_node('movment', anonymous= True)
rate = rospy.Rate(10)  # 10hz
pub = rospy.Publisher('/freicar_commands', FreiCarControl, queue_size=1)
while not rospy.is_shutdown():
    cmd_hl= FreiCarControl()
    cmd_hl.command = str(input())
    cmd_hl.name = str("name")
    print("your input: ", cmd_hl)
    pub.publish(cmd_hl)



