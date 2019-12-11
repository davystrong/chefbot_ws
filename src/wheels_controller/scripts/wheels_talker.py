#!/usr/bin/env python

#test
import rospy
from std_msgs.msg import String, Float32, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


wheel_sep = 0.3
wheel_diameter = 0.09
set_point = 30.0

def wheels_talker():
    rospy.init_node('wheels_talker', anonymous=True)
    # sub = rospy.Subscriber('/joint_states', JointState, handleJointState)
    pub_vel = rospy.Publisher('/wheels_velocidade', Float32, queue_size=5)
    pub_pos = rospy.Publisher('/wheels_posicao', Float32, queue_size=5)
    pub_ang = rospy.Publisher('/wheels_angulo', Float32, queue_size=5)
    # pub.publish(90.0)
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        # rate.sleep()
        print("Tipos: Velocidade - 1, Posicao - 2, Angulo - 3")
        angstr = input('Tipo: ')
        if(angstr == 1):
            #Velocidade
            angstr = input('Velocidade: ')
            valor = float(angstr)
            pub_vel.publish(valor)
        elif(angstr == 2):
            #Posicao
            angstr = input('Posicao: ')
            valor = float(angstr)
            pub_pos.publish(valor)
        elif(angstr == 3):
            #Angulo
            angstr = input('Angulo: ')
            valor = float(angstr)
            pub_ang.publish(valor)



if __name__ == '__main__':
    try:
        wheels_talker()
    except rospy.ROSInterruptException:
        pass
