#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def talker():
    pub_mode = rospy.Publisher('drive_mode', Float64, queue_size=10)
    rospy.init_node('drive_mode', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        mode = -1
        #rospy.loginfo(vel)
        pub_mode.publish(mode)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass