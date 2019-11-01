#!/usr/bin/env python

#test
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5)
    # pub = rospy.Publisher('left_wheel_speed', Float32, queue_size=5)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        # pub.publish(10.0)
        twist = Twist()
        twist.linear.x = 1; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 1
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
