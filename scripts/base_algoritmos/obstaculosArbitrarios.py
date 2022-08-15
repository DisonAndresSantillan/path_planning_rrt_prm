#!/usr/bin/python3
from ast import Break
import rospy 
import time
import random
import numpy as np
from sklearn.neighbors import KDTree
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist,Vector3,Point
from nav_msgs.msg import Odometry
from sklearn import neighbors
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt

#Variables Globales del ROBOT 1
def main():
    global pub_markers
    #inicialización
    rospy.init_node('OBSTACULOS')
    pub_markers = rospy.Publisher('Markers_Obs', Marker, queue_size=10)
    rospy.Rate(20)




class Marcadores(object):

     def Obstaculos(pub_markers):
        pared1=Marker(); pared2=Marker(); pared3=Marker(); pared4=Marker(); pared5=Marker(); pared6=Marker()
        pared7=Marker(); pared8=Marker(); pared9=Marker(); pared9=Marker(); pared10=Marker()
        pared11=Marker(); pared12=Marker(); pared13=Marker()
        obstPared=Marker();obstPared1=Marker();obstPared2=Marker()
        pared1.type =pared1.CUBE
        pared1.header.frame_id="map"
        pared1.id = 0
        pared1.ns ="paredes"
        pared1.lifetime=rospy.Duration()
        pared1.action = pared1.ADD
        pared1.scale.x = 0.3
        pared1.scale.y = 2.5
        pared1.scale.z =0.5
        pared1.pose.position.x = 0.1
        pared1.pose.position.y = 2.9
        pared1.pose.position.z = 0.25
        pared1.pose.orientation.x = 0.0
        pared1.pose.orientation.y = 0.0
        pared1.pose.orientation.z = 0
        pared1.pose.orientation.w = 1
        pared1.color.a = 1
        pared1.color.r = pared1.color.g = pared1.color.b = 6.6

        pared2.type =pared2.CUBE
        pared2.header.frame_id="map"
        pared2.id = 1
        pared2.ns ="paredes"
        pared2.lifetime=rospy.Duration()
        pared2.action = pared2.ADD
        pared2.scale.x = 0.3
        pared2.scale.y = 2.9
        pared2.scale.z =0.5
        pared2.pose.position.x = 1.8
        pared2.pose.position.y = 0.55
        pared2.pose.position.z = 0.25
        pared2.pose.orientation.x = 0.0
        pared2.pose.orientation.y = 0.0
        pared2.pose.orientation.z = 0
        pared2.pose.orientation.w = 1
        pared2.color.a = 1
        pared2.color.r = pared2.color.g = pared2.color.b = 6.6

        
        pared3.type =pared3.CUBE
        pared3.header.frame_id="map"
        pared3.id = 2
        pared3.ns ="paredes"
        pared3.lifetime=rospy.Duration()
        pared3.action = pared3.ADD
        pared3.scale.x = 2.6
        pared3.scale.y = 0.3
        pared3.scale.z =0.5
        pared3.pose.position.x = 3
        pared3.pose.position.y = 1.85
        pared3.pose.position.z = 0.25
        pared3.pose.orientation.x = 0.0
        pared3.pose.orientation.y = 0.0
        pared3.pose.orientation.z = 0
        pared3.pose.orientation.w = 1
        pared3.color.a = 1
        pared3.color.r = pared3.color.g = pared3.color.b = 6.6


        pared4.type =pared4.CUBE
        pared4.header.frame_id="map"
        pared4.id = 3
        pared4.ns ="paredes"
        pared4.lifetime=rospy.Duration()
        pared4.action = pared4.ADD
        pared4.scale.x = 1.4
        pared4.scale.y = 0.3
        pared4.scale.z = 0.5
        pared4.pose.position.x = 1
        pared4.pose.position.y = 0.1
        pared4.pose.position.z = 0.25
        pared4.pose.orientation.x = 0.0
        pared4.pose.orientation.y = 0.0
        pared4.pose.orientation.z = 0
        pared4.pose.orientation.w = 1
        pared4.color.a = 1
        pared4.color.r = pared4.color.g = pared4.color.b = 6.6


        pared5.type =pared5.CUBE
        pared5.header.frame_id="map"
        pared5.id = 4
        pared5.ns ="paredes"
        pared5.lifetime=rospy.Duration()
        pared5.action = pared5.ADD
        pared5.scale.x = 0.3
        pared5.scale.y = 1.6
        pared5.scale.z = 0.5
        pared5.pose.position.x = -1.75
        pared5.pose.position.y = 1.25
        pared5.pose.position.z = 0.25
        pared5.pose.orientation.x = 0.0
        pared5.pose.orientation.y = 0.0
        pared5.pose.orientation.z = 0
        pared5.pose.orientation.w = 1
        pared5.color.a = 1
        pared5.color.r = pared5.color.g = pared5.color.b = 6.6


        pared6.type =pared6.CUBE
        pared6.header.frame_id="map"
        pared6.id = 5
        pared6.ns ="paredes"
        pared6.lifetime=rospy.Duration()
        pared6.action = pared6.ADD
        pared6.scale.x = 2.3
        pared6.scale.y = 0.3
        pared6.scale.z = 0.5
        pared6.pose.position.x = -2.75
        pared6.pose.position.y = 2
        pared6.pose.position.z = 0.25
        pared6.pose.orientation.x = 0.0
        pared6.pose.orientation.y = 0.0
        pared6.pose.orientation.z = 0
        pared6.pose.orientation.w = 1
        pared6.color.a = 1
        pared6.color.r = pared6.color.g = pared6.color.b = 6.6


        pared7.type =pared7.CUBE
        pared7.header.frame_id="map"
        pared7.id = 6
        pared7.ns ="paredes"
        pared7.lifetime=rospy.Duration()
        pared7.action = pared7.ADD
        pared7.scale.x = 0.3
        pared7.scale.y = 1.5
        pared7.scale.z = 0.5
        pared7.pose.position.x = -2.7
        pared7.pose.position.y = 2.6
        pared7.pose.position.z = 0.25
        pared7.pose.orientation.x = 0.0
        pared7.pose.orientation.y = 0.0
        pared7.pose.orientation.z = 0
        pared7.pose.orientation.w = 1
        pared7.color.a = 1
        pared7.color.r = pared7.color.g = pared7.color.b = 6.6



        pared8.type =pared8.CUBE
        pared8.header.frame_id="map"
        pared8.id = 7
        pared8.ns ="paredes"
        pared8.lifetime=rospy.Duration()
        pared8.action = pared8.ADD
        pared8.scale.x = 2.2
        pared8.scale.y = 0.3
        pared8.scale.z = 0.5
        pared8.pose.position.x = -2.7
        pared8.pose.position.y = -1.3
        pared8.pose.position.z = 0.25
        pared8.pose.orientation.x = 0.0
        pared8.pose.orientation.y = 0.0
        pared8.pose.orientation.z = 0
        pared8.pose.orientation.w = 1
        pared8.color.a = 1
        pared8.color.r = pared8.color.g = pared8.color.b = 6.6


        pared9.type =pared9.CUBE
        pared9.header.frame_id="map"
        pared9.id = 8
        pared9.ns ="paredes"
        pared9.lifetime=rospy.Duration()
        pared9.action = pared9.ADD
        pared9.scale.x = 0.3
        pared9.scale.y = 1.7
        pared9.scale.z = 0.5
        pared9.pose.position.x = -3.75
        pared9.pose.position.y = -0.6
        pared9.pose.position.z = 0.25
        pared9.pose.orientation.x = 0.0
        pared9.pose.orientation.y = 0.0
        pared9.pose.orientation.z = 0
        pared9.pose.orientation.w = 1
        pared9.color.a = 1
        pared9.color.r = pared9.color.g = pared9.color.b = 6.6


        pared10.type =pared10.CUBE
        pared10.header.frame_id="map"
        pared10.id = 9
        pared10.ns ="paredes"
        pared10.lifetime=rospy.Duration()
        pared10.action = pared10.ADD
        pared10.scale.x = 0.3
        pared10.scale.y = 1.7
        pared10.scale.z = 0.5
        pared10.pose.position.x = -1.75
        pared10.pose.position.y = -2.1
        pared10.pose.position.z = 0.25
        pared10.pose.orientation.x = 0.0
        pared10.pose.orientation.y = 0.0
        pared10.pose.orientation.z = 0
        pared10.pose.orientation.w = 1
        pared10.color.a = 1
        pared10.color.r = pared10.color.g = pared10.color.b = 6.6

        pared11.type =pared11.CUBE
        pared11.header.frame_id="map"
        pared11.id = 10
        pared11.ns ="paredes"
        pared11.lifetime=rospy.Duration()
        pared11.action = pared11.ADD
        pared11.scale.x = 0.3
        pared11.scale.y = 2.1
        pared11.scale.z = 0.5
        pared11.pose.position.x = 0.5
        pared11.pose.position.y = -3.6
        pared11.pose.position.z = 0.25
        pared11.pose.orientation.x = 0.0
        pared11.pose.orientation.y = 0.0
        pared11.pose.orientation.z = 0
        pared11.pose.orientation.w = 1
        pared11.color.a = 1
        pared11.color.r = pared11.color.g = pared11.color.b = 6.6

        pared12.type =pared12.CUBE
        pared12.header.frame_id="map"
        pared12.id = 11
        pared12.ns ="paredes"
        pared12.lifetime=rospy.Duration()
        pared12.action = pared12.ADD
        pared12.scale.x = 2.5
        pared12.scale.y = 0.3
        pared12.scale.z = 0.5
        pared12.pose.position.x = 1.6
        pared12.pose.position.y = -2.65
        pared12.pose.position.z = 0.25
        pared12.pose.orientation.x = 0.0
        pared12.pose.orientation.y = 0.0
        pared12.pose.orientation.z = 0
        pared12.pose.orientation.w = 1
        pared12.color.a = 1
        pared12.color.r = pared12.color.g = pared12.color.b = 6.6

        pared13.type =pared13.CUBE
        pared13.header.frame_id="map"
        pared13.id = 12
        pared13.ns ="paredes"
        pared13.lifetime=rospy.Duration()
        pared13.action = pared13.ADD
        pared13.scale.x = 0.3
        pared13.scale.y = 1.1
        pared13.scale.z = 0.5
        pared13.pose.position.x = 2.75
        pared13.pose.position.y = -3.05
        pared13.pose.position.z = 0.25
        pared13.pose.orientation.x = 0.0
        pared13.pose.orientation.y = 0.0
        pared13.pose.orientation.z = 0
        pared13.pose.orientation.w = 1
        pared13.color.a = 1
        pared13.color.r = pared13.color.g = pared13.color.b = 6.6

        obstPared.type =obstPared.CUBE
        obstPared.header.frame_id="map"
        obstPared.id = 1
        obstPared.ns ="Obstaculos"
        obstPared.lifetime=rospy.Duration()
        obstPared.action = obstPared.ADD
        obstPared.scale.x = 1.5
        obstPared.scale.y = 0.7
        obstPared.scale.z = 0.5
        obstPared.pose.position.x = 0.2
        obstPared.pose.position.y = -1.5
        obstPared.pose.position.z = 0.25
        obstPared.pose.orientation.x = 0.0
        obstPared.pose.orientation.y = 0.0
        obstPared.pose.orientation.z = 0
        obstPared.pose.orientation.w = 1
        obstPared.color.a = 1
        obstPared.color.r = 255/255
        obstPared.color.g = 200/255
        obstPared.color.b = 0/255

        obstPared1.type =obstPared1.CUBE
        obstPared1.header.frame_id="map"
        obstPared1.id = 2
        obstPared1.ns ="Obstaculos"
        obstPared1.lifetime=rospy.Duration()
        obstPared1.action = obstPared1.ADD
        obstPared1.scale.x = 0.6
        obstPared1.scale.y = 0.6
        obstPared1.scale.z = 0.5
        obstPared1.pose.position.x = -0.5
        obstPared1.pose.position.y = 0.9
        obstPared1.pose.position.z = 0.25
        obstPared1.pose.orientation.x = 0.0
        obstPared1.pose.orientation.y = 0.0
        obstPared1.pose.orientation.z = 0
        obstPared1.pose.orientation.w = 1
        obstPared1.color.a = 1
        obstPared1.color.r = 255/255
        obstPared1.color.g = 200/255
        obstPared1.color.b = 0/255

        obstPared2.type =obstPared2.CUBE
        obstPared2.header.frame_id="map"
        obstPared2.id = 3
        obstPared2.ns ="Obstaculos"
        obstPared2.lifetime=rospy.Duration()
        obstPared2.action = obstPared2.ADD
        obstPared2.scale.x = 1
        obstPared2.scale.y = 0.3
        obstPared2.scale.z = 0.5
        obstPared2.pose.position.x = -2
        obstPared2.pose.position.y = -0.5
        obstPared2.pose.position.z = 0.25
        obstPared2.pose.orientation.x = 0.0
        obstPared2.pose.orientation.y = 0.0
        obstPared2.pose.orientation.z = 0
        obstPared2.pose.orientation.w = 1
        obstPared2.color.a = 1
        obstPared2.color.r = 255/255
        obstPared2.color.g = 200/255
        obstPared2.color.b = 0/255

        pub_markers.publish(pared1)
        pub_markers.publish(pared2)
        pub_markers.publish(pared3)
        pub_markers.publish(pared4)
        pub_markers.publish(pared5)
        pub_markers.publish(pared6)
        pub_markers.publish(pared7)
        pub_markers.publish(pared8)
        pub_markers.publish(pared9)
        pub_markers.publish(pared10)
        pub_markers.publish(pared11)
        pub_markers.publish(pared12)
        pub_markers.publish(pared13)
        #pub_markers.publish(obstPared)
        #pub_markers.publish(obstPared1)
        #pub_markers.publish(obstPared2)



if __name__=='__main__':
    try:
        while(True):
            main()
            Marcadores.Obstaculos(pub_markers)

    except rospy.ROSInterruptException:
        pass