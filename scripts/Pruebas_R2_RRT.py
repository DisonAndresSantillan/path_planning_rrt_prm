#!/usr/bin/python3
"""
Author: Edison Andrés Santillán 

"""
import rospy 
from geometry_msgs.msg import Point
import Ruta_RRT
sigma=0.9

#Funcion principal
def main():
    
    pR1meta=Point()
    pR1meta.x=-2.8;pR1meta.y=-2.3
    robot=2
    prueba=3
    N=1000
    #sigma=0.8
    Ruta_RRT.AlgoritmoRRT.runRRT(N,pR1meta,sigma,robot,prueba)
if __name__=='__main__':
    try:
        s=1
        while(s!=0):
            main()
            s=int(input("1. Continuar, 0.Salir: "))
            if(s==0):
                break
            print("Sigma es: ",sigma)
            sigma=float(input("ingrese sigma [0.1 0.9]: "))
            if(sigma>=0.9):
                sigma=0.9
            elif(sigma<=0.1):
                sigma=0.1


    except rospy.ROSInterruptException:
        pass