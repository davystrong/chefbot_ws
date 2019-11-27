#!/usr/bin/env python

import trollius
from trollius import From

import pygazebo
import pygazebo.msg.joint_cmd_pb2
import rospy
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import JointState

wheel_sep = 0.3
wheel_diameter = 0.09

ave_speed_sp = 1.0
turn_angle_sp = 0.0
left_wheel_rot = 0.0
right_wheel_rot = 0.0
wheel_position_left = 0.0
wheel_position_right = 0.0


# class LeftMessage(pygazebo.msg.joint_cmd_pb2.JointCmd):
#     def __init__(self, force, cls, name, bases, dictionary):
#         super(LeftMessage, cls).__init__(name, bases, dictionary)
#         self.name = 'mobile_base::wheel_left_joint'
#         self.axis = 0
#         self.force = force

# These should both be objects inheriting from JointCmd but I didn't manage
def LeftMessage(force):
    message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    message.name = 'mobile_base::wheel_left_joint'
    message.axis = 0
    message.force = force
    return message


def RightMessage(force):
    message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    message.name = 'mobile_base::wheel_right_joint'
    message.axis = 0
    message.force = force
    return message


@trollius.coroutine
def publish_loop():
    manager = yield From(pygazebo.connect())

    publisher = yield From(
        manager.advertise('/gazebo/default/mobile_base/joint_cmd',
                          'gazebo.msgs.JointCmd'))

    # Python 2 limitation. In python 3 I'd define the variable here and use nonlocal
    # If anyone thinks of a better way, let me know
    global ave_speed_sp
    global turn_angle_sp

    def handleAveSpeedSp(new_speed):
        global ave_speed_sp
        ave_speed_sp = new_speed
        print(new_speed)

    def handleTurnAngleSp(new_angle):
        global turn_angle_sp
        turn_angle_sp = new_angle
        print(new_angle)

    def handleJointState(joint_state):
        global left_wheel_rot, right_wheel_rot
        global wheel_position_left, wheel_position_right

        left_wheel_rot, right_wheel_rot = joint_state.velocity
        wheel_position_left, wheel_position_right = joint_state.position

        error_left_speed = left_wheel_rot*(wheel_diameter/2) - ave_speed_sp
        error_right_speed = right_wheel_rot*(wheel_diameter/2) - ave_speed_sp

        ul = error_left_speed*1
        ur = error_right_speed*1

        print('testing')

        yield From(publisher.publish(LeftMessage(ul)))
        yield From(publisher.publish(RightMessage(ur)))

    rospy.Subscriber(
        '/david/speed', Float32, handleAveSpeedSp)

    rospy.Subscriber(
        '/david/turn_angle', Float32, handleTurnAngleSp)

    print(1)
    manager.subscribe('/gazebo/default/mobile_base/joint_state',
                      'gazebo.msgs.JointState', handleJointState)
    print(2)

    # rospy.Subscriber('/joint_states', JointState, handleJointState)

    # left_message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    # message.name = 'mobile_base::wheel_left_joint'
    # message.axis = 0
    # message.force = -0.0

    while True:
        pass
        # yield From(publisher.publish(LeftMessage(0.1)))
        # yield From(publisher.publish(RightMessage(0.1)))
        # yield From(trollius.sleep(1.0))
        # print('test')


loop = trollius.get_event_loop()
loop.run_until_complete(publish_loop())
