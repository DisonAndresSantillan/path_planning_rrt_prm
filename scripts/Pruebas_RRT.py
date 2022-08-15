#!/usr/bin/python3
"""
Author: Edison Andrés Santillán 

"""
from ast import parse
import rospy 
from geometry_msgs.msg import Point
import Ruta_RRT
import argparse
global theta_rob;xMin=-6;xMax=6;yMin=-6;yMax=6
pmeta=Point()
pmeta.x=100;pmeta.y=100
#Funcion principal
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("nIt", type=int)
    parser.add_argument("robot", type=int)
    parser.add_argument("prueba", type=int)
    parser.add_argument("sigma", type=float)
    args=parser.parse_args()
    N=args.nIt
    prueba=args.prueba
    robot=args.robot
    sigma=args.sigma

    if(prueba==1):
        if(robot==1):
            print("****************************************************************************************************")
            print("                           PRUEBA 1, EJECUTANDO ALGORITMO RRT ROBOT 1                            ")
            print("****************************************************************************************************")
            m=input("\nPor favor configure la posición de salida del robot con los punteros (2D Nav Goal) de Rviz y posteriormente presione ENTER para continuar... ")     
            while(not fueraMapa(pmeta)):
                pmeta.x=float(input("Ingrese la coordenada X de Llegada: "))
                pmeta.y=float(input("Ingrese la coordenada Y de Llegada: "))
                if(not fueraMapa(pmeta)):
                    print("Coordenada fuera del rango")
            Ruta_RRT.AlgoritmoRRT.runRRT(N,pmeta,sigma,robot,prueba)

        if(robot==2):
            print("****************************************************************************************************")
            print("                           PRUEBA 1, EJECUTANDO ALGORITMO RRT ROBOT 2                           ")
            print("****************************************************************************************************")
            m=input("\nPor favor configure la posición de salida del robot con los punteros (2D Nav Goal) de Rviz y posteriormente presione ENTER para continuar... ")    
            while(not fueraMapa(pmeta)):
                pmeta.x=float(input("Ingrese la coordenada X de Llegada: "))
                pmeta.y=float(input("Ingrese la coordenada Y de Llegada: "))
                if(not fueraMapa(pmeta)):
                    print("Coordenada fuera del rango")
            Ruta_RRT.AlgoritmoRRT.runRRT(N,pmeta,sigma,robot,prueba)

    if(prueba<=777 and prueba>=111):
        if(robot==1):
            print("****************************************************************************************************")
            print("                           PRUEBA 2, EJECUTANDO ALGORITMO RRT ROBOT 1                           ")
            print("****************************************************************************************************")
            Ruta_RRT.AlgoritmoRRT.runRRT(N,pmeta,sigma,robot,prueba)

        if(robot==2):
            print("****************************************************************************************************")
            print("                           PRUEBA 2, EJECUTANDO ALGORITMO RRT ROBOT 2                           ")
            print("****************************************************************************************************")
            Ruta_RRT.AlgoritmoRRT.runRRT(N,pmeta,sigma,robot,prueba)
        

    if(prueba==3):
        if(robot==1):
            print("****************************************************************************************************")
            print("                           PRUEBA 3, EJECUTANDO ALGORITMO RRT ROBOT 1                           ")
            print("****************************************************************************************************")
            Ruta_RRT.AlgoritmoRRT.runRRT(N,pmeta,sigma,robot,prueba)

        if(robot==2):
            print("****************************************************************************************************")
            print("                           PRUEBA 3, EJECUTANDO ALGORITMO RRT ROBOT 2                           ")
            print("****************************************************************************************************")
            Ruta_RRT.AlgoritmoRRT.runRRT(N,pmeta,sigma,robot,prueba)

def fueraMapa(pNuevo):
    if((pNuevo.x<xMax and pNuevo.x>xMin) and (pNuevo.y<yMax and pNuevo.y>yMin)):
        return True
    else:
        return False

if __name__=='__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass