#!/usr/bin/env python

#test
import rospy
from std_msgs.msg import String, Float32, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math


wheel_sep = 0.3
wheel_diameter = 0.09
set_point = 30.0
straightVelocity = 0.0
turnVelocity = 0.0
in_use = 0
x_a_moved = 0
y_a_moved = 0

pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)

def movementCallback(Odometer):
    global x_a_moved
    global y_a_moved
    global angle_Yaw
    global img
    quaternion = (
        Odometer.pose.pose.orientation.x,
        Odometer.pose.pose.orientation.y,
        Odometer.pose.pose.orientation.z,
        Odometer.pose.pose.orientation.w)
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)
    angle_Yaw = yaw
    #print(yaw)
    x_a_moved = Odometer.pose.pose.position.x
    y_a_moved = Odometer.pose.pose.position.y

def handleJointState(jointState):
    #print(jointState.velocity)
    rospy.sleep(0.1)

def handleWheels_velocidade(valor):
    global in_use
    global straightVelocity

    while(in_use):
        rospy.sleep(0.01)
    in_use = 1

    #print("Eu recebi: ")
    #print(valor.data)
    straightVelocity = valor.data

    in_use = 0

def handleWheels_posicao(valor):
    global x_a_moved
    global y_a_moved
    global in_use
    global straightVelocity

    while(in_use):
        rospy.sleep(0.01)
    in_use = 1

    x_inicial = x_a_moved
    y_inicial = y_a_moved

    x_ponto_dist = x_a_moved - x_inicial
    y_ponto_dist = y_a_moved - y_inicial

    while(math.sqrt((x_ponto_dist*x_ponto_dist)+(y_ponto_dist*y_ponto_dist))<math.fabs(valor.data)):
        straightVelocity = 0.5 * (valor.data/math.fabs(valor.data))
        x_ponto_dist = math.fabs(x_a_moved - x_inicial)
        y_ponto_dist = math.fabs(y_a_moved - y_inicial)
        #print(math.sqrt((x_ponto_dist*x_ponto_dist)+(y_ponto_dist*y_ponto_dist)))
    straightVelocity = 0

    in_use = 0

def handleWheels_angulo(valor):
    global in_use
    global turnVelocity
    global angle_Yaw

    while(in_use):
        rospy.sleep(0.01)
    in_use = 1

    valorRad = (valor.data*math.pi)/180

    if (angle_Yaw + valorRad)>math.pi:
        target_angle = (angle_Yaw+valorRad)-(2*math.pi)
    elif (angle_Yaw + valorRad)<(0-math.pi):
        target_angle = (angle_Yaw+valorRad)+(2*math.pi)
    else:
        target_angle = angle_Yaw+valorRad

    turnSpeed = 1
    tolerancia = 0.05

    if(valor.data != 0):
        while((angle_Yaw < (target_angle - tolerancia)) or (angle_Yaw > (target_angle + tolerancia))):
            sinal_atual = (angle_Yaw-target_angle)/math.fabs(angle_Yaw-target_angle)
            turnVelocity = 1 * (valor.data/math.fabs(valor.data))
            #print((target_angle*180)/math.pi)
            #print((angle_Yaw*180)/math.pi)
    turnVelocity = 0

    in_use = 0

def wheels_control():
    global straightVelocity
    global turnVelocity

    control_speed = 0
    control_turn = 0

    rospy.init_node('wheels_control', anonymous=True)
    rospy.Subscriber('/wheels_velocidade', Float32, handleWheels_velocidade)
    rospy.Subscriber('/wheels_posicao', Float32, handleWheels_posicao)
    rospy.Subscriber('/wheels_angulo', Float32, handleWheels_angulo)
    rospy.Subscriber('/joint_states', JointState, handleJointState)
    rospy.Subscriber('/odom', Odometry, movementCallback)

    while(1):
        if straightVelocity > control_speed:
            control_speed = min( straightVelocity, control_speed + 0.02 )
        elif straightVelocity < control_speed:
            control_speed = max( straightVelocity, control_speed - 0.02 )
        else:
            control_speed = straightVelocity

        if turnVelocity > control_turn:
            control_turn = min( turnVelocity, control_turn + 0.1 )
        elif turnVelocity < control_turn:
            control_turn = max( turnVelocity, control_turn - 0.1 )
        else:
            control_turn = turnVelocity

        twist = Twist()
        twist.linear.x = straightVelocity; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turnVelocity
        pub.publish(twist)
    # pub.publish(90.0)
    rospy.spin()


if __name__ == '__main__':
    try:
        wheels_control()
    except rospy.ROSInterruptException:
        pass
