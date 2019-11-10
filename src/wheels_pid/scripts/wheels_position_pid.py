#!/usr/bin/env python

#test
import rospy
from std_msgs.msg import String, Float32, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# <wheel_separation>.30</wheel_separation>
# <wheel_diameter>0.09</wheel_diameter>

# wheel_speed_cmd_[LEFT] = msg->linear.x - msg->angular.z * (wheel_sep_) / 2;
# wheel_speed_cmd_[RIGHT] = msg->linear.x + msg->angular.z * (wheel_sep_) / 2;

wheel_sep = 0.3
wheel_diameter = 0.09


pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

def position_control(set_point_position, wheel_position, ki, kp):
    error = set_point_position - wheel_position
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

def forward_movement(position_set_point, joint_state):
    left_wheel_rot, right_wheel_rot = joint_state.velocity
    wheel_position_left, wheel_position_right = joint_state.position

    if(forward_movement.endPositionLeft == 0 and forward_movement.endPositionRight == 0 ):
        forward_movement.endPositionLeft = wheel_position_left + position_set_point
        forward_movement.endPositionRight = wheel_position_right + position_set_point

    velocity_set_left =  position_control(forward_movement.endPositionLeft, wheel_position_left, 0.0002, 0.8)
    velocity_set_right = position_control(forward_movement.endPositionRight, wheel_position_right, 0.0002, 0.8)

    desired_left_speed = calc_control(left_wheel_rot, velocity_set_left, 0.2, 4, 0) * wheel_diameter / 2
    desired_right_speed = calc_control(right_wheel_rot, velocity_set_right, 0.2, 4, 0) * wheel_diameter / 2
    twist = getTwist(desired_left_speed, desired_right_speed)
    pub.publish(twist)
forward_movement.endPositionLeft = 0
forward_movement.endPositionRight = 0

def turn_left_movement(angle_set_point, joint_state):
    left_wheel_rot, right_wheel_rot = joint_state.velocity
    wheel_position_left, wheel_position_right = joint_state.position

    if(turn_left_movement.endPositionLeft == 0 and turn_left_movement.endPositionRight == 0 ):
        turn_left_movement.endPositionLeft = wheel_position_left - angle_set_point * (wheel_sep * 4 / (180 * wheel_diameter))
        turn_left_movement.endPositionRight = wheel_position_right + angle_set_point * (wheel_sep * 4 / (180 * wheel_diameter))

    velocity_set_left =  position_control(turn_left_movement.endPositionLeft,wheel_position_left, 0.001, 1)
    velocity_set_right =  position_control(turn_left_movement.endPositionRight,wheel_position_right, 0.001, 1)

    desired_left_speed = calc_control(left_wheel_rot, velocity_set_left, 1, 4, 0) * wheel_diameter / 2
    desired_right_speed = calc_control(right_wheel_rot, velocity_set_right, 1, 4, 0) * wheel_diameter / 2
    twist = getTwist(desired_left_speed, desired_right_speed)
    pub.publish(twist)
turn_left_movement.endPositionLeft = 0
turn_left_movement.endPositionRight = 0


def turn_right_movement(angle_set_point, joint_state):
    left_wheel_rot, right_wheel_rot = joint_state.velocity
    wheel_position_left, wheel_position_right = joint_state.position

    if(turn_right_movement.endPositionLeft == 0 and turn_right_movement.endPositionRight == 0 ):
        turn_right_movement.endPositionLeft = wheel_position_left + angle_set_point * (wheel_sep * 4 / (180 * wheel_diameter))
        turn_right_movement.endPositionRight = wheel_position_right - angle_set_point * (wheel_sep * 4 / (180 * wheel_diameter))

    velocity_set_left =  position_control(turn_right_movement.endPositionLeft,wheel_position_left, 0.001, 1)
    velocity_set_right =  position_control(turn_right_movement.endPositionRight,wheel_position_right, 0.001, 1)

    desired_left_speed = calc_control(left_wheel_rot, velocity_set_left, 10, 4, 0) * wheel_diameter / 2
    desired_right_speed = calc_control(right_wheel_rot, velocity_set_right, 10, 4, 0) * wheel_diameter / 2
    twist = getTwist(desired_left_speed, desired_right_speed)
    pub.publish(twist)
turn_right_movement.endPositionLeft = 0
turn_right_movement.endPositionRight = 0


def handleJointState(joint_state):
    if (handleJointState.set_point > 0):
        turn_left_movement(handleJointState.set_point, joint_state)
    else:
        turn_right_movement(handleJointState.set_point, joint_state)
handleJointState.set_point = 0

def handleRotateSp(angle):
    handleJointState.set_point = angle.data
    print(angle.data)

def talker():

    rospy.init_node('talker', anonymous=True)
    sub = rospy.Subscriber('/joint_states', JointState, handleJointState)
    rotate_sp_sub = rospy.Subscriber('/rotate_angle', Float32, handleRotateSp)
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
