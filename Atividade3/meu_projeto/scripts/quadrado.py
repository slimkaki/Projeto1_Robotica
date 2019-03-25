#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3
from math import pi


v = 0.1 # Velocidade linear
w =(pi/2)*0.1 # Velocidade angular

#t0=rospy.Time.now().to_sec()
#c=0

if __name__ == "__main__":
    rospy.init_node("roda_exemplo")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            print("parei")
            pub.publish(vel)
            rospy.sleep(2.0)

            vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
            print("to andano")
            pub.publish(vel)
            rospy.sleep(6.0)
            
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            print("parei")
            pub.publish(vel)
            rospy.sleep(2.0)
            
            vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
            print("girando")
            pub.publish(vel)
            rospy.sleep(10.0)

            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            print("parei")
            pub.publish(vel)
            rospy.sleep(2.0)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")