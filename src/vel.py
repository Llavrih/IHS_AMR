#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def talker():
    pub_vel = rospy.Publisher('cmd_vel', Float64, queue_size=10)
    rospy.init_node('cmd_vel', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        vel = 0.2
        #rospy.loginfo(vel)
        pub_vel.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
