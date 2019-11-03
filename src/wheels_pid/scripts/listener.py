#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sensor_msgs
from sensor_msgs.msg import JointState


def actuatorCallback(jointState):
    # here you run your PID control, each time the subscriber collects data
    rospy.loginfo('\r\nleft speed is: %f, \r\nrigth speed is %f',
                  jointState.velocity[0],
                  jointState.velocity[1])

def listener(name):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(name, JointState, actuatorCallback)

    # spin() simply keeps python from exiting until this node is stopped

    rospy.spin()

if __name__ == '__main__':
    listener('joint_states')
