#!/usr/bin/python3
"""
Algorithm path planning Probabilistic Rad Map (PRM) ROS noetic
Author: Edison Andrés Santillán 

"""

import rospy 
from geometry_msgs.msg import Point
import Ruta_PRM


#Funcion principal
def main():

    pR2meta=Point()
    pR2meta.x=-3;pR2meta.y=3
    #pR2meta.x=-3;pR2meta.y=1

    robot=1
    prueba=3
    N=1000
    Ruta_PRM.AlgoritmoPRM.runPRM(N,pR2meta,robot,prueba)


if __name__=='__main__':
    try:
        s=1
        while(s!=0):
            main()
            #Elimino los marcadores
            s=int(input("1. Continuar, 0.Salir: "))
            if(s==0):
                break

    except rospy.ROSInterruptException:
        pass