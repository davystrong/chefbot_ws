#!/usr/bin/env python

#test
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# <wheel_separation>.30</wheel_separation>
# <wheel_diameter>0.09</wheel_diameter>

# wheel_speed_cmd_[LEFT] = msg->linear.x - msg->angular.z * (wheel_sep_) / 2;
# wheel_speed_cmd_[RIGHT] = msg->linear.x + msg->angular.z * (wheel_sep_) / 2;

wheel_sep = 0.3
wheel_diameter = 0.09
set_point = 100.0

pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

def position_control(wheel_position, ki, kp):
    error = set_point - wheel_position
    position_control.integrator += error/100

    return error * kp + position_control.integrator * ki

position_control.integrator = 0

def getTwist(left_wheel_speed, right_wheel_speed):
    twist = Twist()
    twist.linear.x = (right_wheel_speed + left_wheel_speed) / 2
    twist.angular.z = (right_wheel_speed - left_wheel_speed) / wheel_sep
    return twist

def calc_control(wheel_rot, velocity_setpoint, kp, ki, kd):
    error = velocity_setpoint - wheel_rot
    calc_control.integrator += error/100 #100 is the update rate

    output = error * kp + calc_control.integrator * ki

    return output
calc_control.integrator = 0

def handleJointState(joint_state):
    left_wheel_rot, right_wheel_rot = joint_state.velocity
    wheel_position_left, wheel_position_rigth = joint_state.position

    velocity_set_left =  position_control(wheel_position_left, 0.0002, 0.8)
    velocity_set_rigth =  position_control(wheel_position_rigth, 0.0002, 0.8)

    desired_left_speed = calc_control(left_wheel_rot, velocity_set_left, 0.2, 4, 0) * wheel_diameter / 2
    desired_right_speed = calc_control(right_wheel_rot, velocity_set_rigth, 0.2, 4, 0) * wheel_diameter / 2
    twist = getTwist(desired_left_speed, desired_right_speed)
    pub.publish(twist)


def talker():

    rospy.init_node('talker', anonymous=True)
    sub = rospy.Subscriber('/joint_states', JointState, handleJointState)
    # rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        pass
        # twist = Twist()
        # twist.linear.x = 5; twist.linear.y = 0; twist.linear.z = 0
        # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        # pub.publish(twist)
        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
