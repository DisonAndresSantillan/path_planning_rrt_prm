#!/usr/bin/python3
"""
Algorithm path planning Probabilistic Rad Map (PRM) ROS noetic
Author: Edison Andrés Santillán 

"""

import rospy 
import random
import numpy as np
from sklearn.neighbors import KDTree
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

#Variables Globales del ROBOT 1
global theta_r1;xMin=-5;xMax=5;yMin=-5;yMax=5
posRobot=Point();pSalida=Point(); pLlegada=Point()
pSaved={}
pRutaLibre={}
pObst=[]

pLlegada.x=-3;pLlegada.y=0;pSalida.x=3.7;pSalida.y=3.3
N=1000


class AlgoritmoPRM(object):

    def __init__(self, pInit, pMeta):
        self.pInit=pInit
        self.pMeta=pMeta

    #distancia entre el punto cercano y el punto ramdom
    def distanciaEuclideana(p1,p2):
        """
        Entrada:
            p1 es el punto cercano
            p2 es el punto random
        Salida:
            Distancia
        """
        dist=math.sqrt(math.pow(p2.x -p1.x,2)+math.pow(p2.y-p1.y,2))
        return dist



    #Defino un punto random
    def pRand(pObst):
        """
        Entrada:
            Configuración x,y maximos del mapa
        Salida:
            Punto random dentro del mapa
        """
        p=Point()
        while(True):
            #Determino el punto random alejado de las paredes
            X_izq=Point();X_der=Point();Y_inf=Point();Y_sup=Point()
            factorRandom=0.4
            p.x=random.uniform(xMin,xMax)
            p.y=random.uniform(yMin,yMax)
            X_izq.x=p.x-factorRandom;X_izq.y=p.y
            X_der.x=p.x+factorRandom;X_der.y=p.y
            Y_inf.y=p.y-factorRandom;Y_inf.x=p.x
            Y_sup.y=p.y+factorRandom;Y_sup.x=p.x
            if(not AlgoritmoPRM.ColisionaParedes(X_izq,Y_sup,pObst) and not AlgoritmoPRM.ColisionaParedes(Y_sup,X_der,pObst) and not AlgoritmoPRM.ColisionaParedes(X_der,Y_inf,pObst) and not AlgoritmoPRM.ColisionaParedes(Y_inf,X_izq,pObst)and AlgoritmoPRM.fueraMapa(p)):
                break
            
        return p

    
    #Determino el punto mas cercano 
    def randomPneast(pRandom,pSaved,pObst):
        """
        Entrada:
            pRandom: punto random
            pSaved: son los puntos guardados que anteriormente fueron random
        Salida:
            3 puntos (x,y) mas cercano al punto Random
        """
        p=Point()
        pReturn=np.zeros((4,2))
        indice=sorted(pSaved.keys())
        if(len(indice)>1):
            distanciasPuntos={}
            distanciasPuntos.clear()
            for i in indice:
                #Accedo a los datos dentro del diccionario
                p.x=pSaved[i][0];p.y=pSaved[i][1]
                d=AlgoritmoPRM.distanciaEuclideana(pRandom,p)
                #A continuacion ordeno el dict conforme la distancia mas cerca
                distanciasPuntos.setdefault(d,(pSaved[i][0],pSaved[i][1]))
                #obtengo la ubicacion de los puntos conforme la menor distancia
            distan=sorted(distanciasPuntos.keys())            
            nodosVecinos=0;nV=0           
            while(nodosVecinos<len(pReturn[:,1])):
                if(nV>=len(distan)-1):
                    break
                p.x=distanciasPuntos[distan[nV]][0]
                p.y=distanciasPuntos[distan[nV]][1]
                if(pRandom.x!=p.x and pRandom.y!=p.y):
                    if(not AlgoritmoPRM.ColisionaParedes(pRandom,p,pObst)):
                        pReturn[nodosVecinos]=(distanciasPuntos[distan[nV]][0],distanciasPuntos[distan[nV]][1])
                        nodosVecinos+=1
                nV+=1
                
        return pReturn



 #Determino el punto mas cercano 
    def pNeastRutaFinal(pRandom,pSaved):
        """
        Entrada:
            pRandom: punto random
            pSaved: son los puntos guardados que anteriormente fueron random
        Salida:
            puntos (x,y) mas cercano 
        """

        p=Point()
        indice=sorted(pSaved.keys())
        if(len(indice)>1):
            #transformo un dict a una tupla 

            #print("*******************indice*****************")
            distanciasPuntos={}
            distanciasPuntos.clear()
            for i in indice:
                #Accedo a los datos dentro del diccionario
                p.x=pSaved[i][0];p.y=pSaved[i][1]
                d=AlgoritmoPRM.distanciaEuclideana(pRandom,p)
                #A continuacion ordeno el dict conforme la distancia mas cerca
                distanciasPuntos.setdefault(d,(pSaved[i][0],pSaved[i][1]))
                #obtengo la ubicacion de los puntos conforme la menor distancia
            distan=sorted(distanciasPuntos.keys())
            #print(distan)
            p.x=distanciasPuntos[distan[0]][0]
            p.y=distanciasPuntos[distan[0]][1]
        return p



    def fueraMapa(pNuevo):
        if((pNuevo.x<xMax and pNuevo.x>xMin) and (pNuevo.y<yMax and pNuevo.y>yMin)):
            return True
        else:
            return False
  

    def ColisionaParedes(p1,p2,pObst):
        paredes=Marker()

        tam=len(pObst)
        for i in range(0,tam-1):
            paredes=pObst[i]
            izq=(paredes.pose.position.x-paredes.scale.x/2)
            der=(paredes.pose.position.x+paredes.scale.x/2)
            abajo=(paredes.pose.position.y-paredes.scale.y/2)
            arriba=(paredes.pose.position.y+paredes.scale.y/2)

            #Comprobar si hay alguna interseccion

            if( AlgoritmoPRM.intersecarLineas(p1,p2,izq,abajo,der,abajo) or AlgoritmoPRM.intersecarLineas(p1,p2,izq,abajo,izq,arriba) or AlgoritmoPRM.intersecarLineas(p1,p2,der,abajo,der,arriba) or AlgoritmoPRM.intersecarLineas(p1,p2,izq,arriba,der,arriba)):
                return True
                
        return False


    def intersecarLineas(p1,p2,x1,y1,x2,y2):

        #determino la distancia de interseccion desde el punto 
        Pa=((x2-x1)*(p1.y-y1)-(y2-y1)*(p1.x-x1))/((y2-y1)*(p2.x-p1.x)-(x2-x1)*(p2.y-p1.y))
        Pb=((p2.x-p1.x)*(p1.y-y1)-(p2.y-p1.y)*(p1.x-x1))/((y2-y1)*(p2.x-p1.x)-(x2-x1)*(p2.y-p1.y))

        #Si Pa y pb entran dentro de 0-1 quiere decir que se intersecan
        if(Pa<=1 and Pa>=0  and  Pb<=1 and Pb>=0):
            #punto de interseccion
            #px=x1+(Pa*(x2-x1)
            #py=y1+(Pa*(y2-y1)
            return True
        return False


        



class Marcadores(object):

    # grafico en RVIZ los puntos random
    def puntosRandom(p1,cont,pub_markers):
        """
        Entrada:
            p1 punto de inicio
            p2 punto de llegada
        Salida:
            Visualizacion en RVIZ
        """
        # configuracion para el punto de inicio
        pNuevos=Marker()
        pNuevos.type=pNuevos.SPHERE
        pNuevos.header.frame_id="map"
        pNuevos.header.stamp=rospy.Time.now()
        pNuevos.ns="puntos random"
        pNuevos.id=cont
        pNuevos.action=pNuevos.ADD
        pNuevos.pose.position.x=p1.x
        pNuevos.pose.position.y=p1.y
        pNuevos.pose.position.z=0
        pNuevos.pose.orientation.x = 0.0
        pNuevos.pose.orientation.y = 0.0
        pNuevos.pose.orientation.z = 0.0
        pNuevos.pose.orientation.w = 0.0
        pNuevos.scale.x = 0.1#0.05
        pNuevos.scale.y = 0.1#0.05
        pNuevos.scale.z = 0.0
        # marker color
        pNuevos.color.a = 1
        pNuevos.color.r = 255/255
        pNuevos.color.g = 0/255
        pNuevos.color.b = 0/255

        pNuevos.points.append(p1)
        pub_markers.publish(pNuevos)



    # grafico los puntos de llegada y salida para el robot 1
    def Meta(p1,p2,pub_markers):
        """
        Entrada:
            p1 punto de inicio
            p2 punto de llegada
        Salida:
            Visualizacion de los puntos en RVIZ
        """
        # configuracion para el punto de inicio
        p_init=Marker()
        p_init.type=p_init.POINTS
        p_init.header.frame_id="map"
        p_init.header.stamp=rospy.Time.now()
        p_init.ns="vertices inicio/llegada"
        # configuracion para el punto de llegada
        p_meta=Marker()
        p_meta.type=p_meta.POINTS
        p_meta.header.frame_id="map"
        p_meta.header.stamp=rospy.Time.now()
        p_meta.ns="vertices inicio/llegada"
        #-----------------------------------------
        p_init.id=0
        p_meta.id=1
        p_init.action=p_init.ADD
        p_meta.action=p_meta.ADD
        # marker color
        p_init.color.a = 1
        p_init.color.g = 1.0
        p_init.scale.x = 0.2
        p_init.scale.y = 0.2

        p_meta.color.a = 1
        p_meta.color.r = 1.0
        p_meta.scale.x = 0.2
        p_meta.scale.y = 0.2

        p_salida=Point()
        p_llegada=Point()
        p_salida.x = p1.x; p_salida.y = p1.y
        p_llegada.x = p2.x; p_llegada.y = p2.y
        p_init.points.append(p_salida)
        p_meta.points.append(p_llegada)
        pub_markers.publish(p_init)
        pub_markers.publish(p_meta)





    def Obstaculos(pub_markers):
        global pObst
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
        obstPared.scale.x = 0.3
        obstPared.scale.y = 1.6
        obstPared.scale.z = 0.5
        obstPared.pose.position.x = 1.8
        obstPared.pose.position.y = -1.7
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
        obstPared1.scale.x = 0.3
        obstPared1.scale.y = 0.5
        obstPared1.scale.z = 0.5
        obstPared1.pose.position.x = -1.75
        obstPared1.pose.position.y = 0.2
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
        obstPared2.scale.x = 2
        obstPared2.scale.y = 0.3
        obstPared2.scale.z = 0.5
        obstPared2.pose.position.x = -0.7
        obstPared2.pose.position.y = 0.1
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
        pub_markers.publish(obstPared)
        pub_markers.publish(obstPared1)
        pub_markers.publish(obstPared2)



        pObst.append(pared1)
        pObst.append(pared2)
        pObst.append(pared3)
        pObst.append(pared4)
        pObst.append(pared5)
        pObst.append(pared6)
        pObst.append(pared7)
        pObst.append(pared8)
        pObst.append(pared9)
        pObst.append(pared10)
        pObst.append(pared11)
        pObst.append(pared12)
        pObst.append(pared13)
        pObst.append(obstPared)
        pObst.append(obstPared1)
        pObst.append(obstPared2)




    def Rutas(p1,p2,pub_makers,cont_id):
        unirPuntos=Marker();vertices=Marker()

        vertices.type=vertices.POINTS
        unirPuntos.type=unirPuntos.LINE_LIST
        unirPuntos.header.frame_id="map"
        unirPuntos.header.stamp=rospy.Time.now()
        unirPuntos.ns="rutasPRM"
        unirPuntos.id=3+cont_id
        unirPuntos.action=unirPuntos.ADD
        unirPuntos.pose.orientation.w=1
        unirPuntos.scale.x=0.018


        unirPuntos.color.r=130/255
        unirPuntos.color.g=0/255
        unirPuntos.color.b=0/255
        
        unirPuntos.color.a=0.5

        unirPuntos.points.append(p1)
        unirPuntos.points.append(p2)
        

        pub_makers.publish(unirPuntos)

    def RutaLibre(p1,p2,pub_makers,cont_id):
        unirPuntos=Marker();vertices=Marker()

        vertices.type=vertices.POINTS
        unirPuntos.type=unirPuntos.LINE_LIST
        unirPuntos.header.frame_id="map"
        unirPuntos.header.stamp=rospy.Time.now()
        unirPuntos.ns="Ruta Final"
        unirPuntos.id=4+cont_id
        unirPuntos.action=unirPuntos.ADD
        unirPuntos.pose.orientation.w=1
        unirPuntos.scale.x=0.06
        unirPuntos.scale.y=0.06
        unirPuntos.scale.z=0.06


        unirPuntos.color.r=255/255
        unirPuntos.color.g=0/255
        unirPuntos.color.b=0/255
        
        unirPuntos.color.a=1

        unirPuntos.points.append(p1)
        unirPuntos.points.append(p2)
        

        pub_makers.publish(unirPuntos)


#Funcion principal
def main():
    global pub_markers
    #inicialización
    rospy.init_node('Algoritmo_PRM_ROBOT1')
    pub_markers = rospy.Publisher('Markers_R1', Marker, queue_size=10)
    rospy.Subscriber('/r1/odom',Odometry,r1PosXY)
    rospy.Rate(20)
    #Limpio los marcadores
    m=Marker()
    m.ns="rutasPRM"
    m.action=m.DELETEALL
    pub_markers.publish(m)
    runPRM(N,pSalida,pLlegada,pub_markers,pSaved)



def r1PosXY(msg):
    
    posRobot.x=msg.pose.pose.position.x
    posRobot.y=msg.pose.pose.position.y
    posRobot.z=0
    rot = msg.pose.pose.orientation
    (roll, pitch, theta_r1) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])



def runPRM(N,pSalida,pLlegada,pub_markers,pSaved):
    global pObst
    pRandom=Point();p1=Point();p2=Point()
    rutaEncontrada=False
    #inicializando la posicion del robot
    pSalida=posRobot
    pLlegada=AlgoritmoPRM.pRand(pObst)
    pSaved.setdefault(0,(pSalida.x,pSalida.y))
    #INICIO EL ALGORITMO PRM
    tInit=rospy.Time.now().to_sec()

    for i in range(1,N):
        rospy.sleep(0.02)
        #defino obstaculos en el entorno de RVIZ
        Marcadores.Meta(pSalida,pLlegada,pub_markers)
        Marcadores.Obstaculos(pub_markers)
        if(not rutaEncontrada):
            pRandom=AlgoritmoPRM.pRand(pObst)
            Marcadores.puntosRandom(pRandom,i,pub_markers)
            if(AlgoritmoPRM.distanciaEuclideana(pRandom,pLlegada)<=1 and not AlgoritmoPRM.ColisionaParedes(pRandom,pLlegada,pObst) and i>20):
                #garantizo el punto de llegada
                pSaved.setdefault(i,(pLlegada.x,pLlegada.y))
                rutaEncontrada=True
            else:
                tPaso=rospy.Time.now().to_sec()-tInit
                print("it: ",i,"\t#nodos: ",i,"\ttiempo: ",tPaso)
                pSaved.setdefault(i,(pRandom.x,pRandom.y))

        #******************************************************************************************************************
        ij=0        
        if(rutaEncontrada):
            #Grafica de las rutas PRM 
            indice=sorted(pSaved.keys())
            for i in indice:
                rospy.sleep(0.005)
                p1.x=pSaved[i][0];p1.y=pSaved[i][1]
                Pg=AlgoritmoPRM.randomPneast(p1,pSaved,pObst)
                for i in range(len(Pg[:,1])):
                    p2.x=Pg[i][0];p2.y=Pg[i][1]
                    if(p1.x!=p2.x and p1.y!=p2.y):
                        if(not AlgoritmoPRM.ColisionaParedes(p1,p2,pObst)):
                            Marcadores.Rutas(p1,p2,pub_markers,ij)
                            ij+=1
                      #*****************************************************************************************
            print("-------------------------------")
            print("CALCULANDO RUTA OPTIMA")
            print("-------------------------------")
            pVecino=Point();pNodo=Point()
            pasar=0
            while(pasar<=1):
                pRutaLibre.clear()
                X=np.zeros((len(pSaved),2))
                for i in range(len(pSaved)):
                    X[i]=(pSaved[i][0],pSaved[i][1])
                tree = KDTree(X, leaf_size=len(pSaved),metric="chebyshev")
                #Direcciono la ruta 
                # cerca de estos puntos conecto los nodos cercanos 
                sentido=np.zeros((len(pSaved),2))
                x1=np.linspace(pSalida.x,pLlegada.x,len(X[:,1]))
                y1=np.linspace(pSalida.y,pLlegada.y,len(X[:,1]))
                for i in range(len(X[:,1])):
                    sentido[i]=(x1[i],y1[i])
                ind = tree.query_radius(sentido, r=10)    

                X=X[ind[0]]
                #Obtengo todos los nodos ordenados
                pSaved.clear()
                for i in range(0,len(X)):
                    pSaved.setdefault(i,(X[len(X)-i-1][0],X[len(X)-i-1][1]))
                    
                #Grafico el origen  
                if(pasar==0):
                    pVecino.x=pLlegada.x;pVecino.y=pLlegada.y
                    pRutaLibre.setdefault(0,(pVecino.x,pVecino.y))
                elif(pasar==1):
                    pVecino.x=pSalida.x;pVecino.y=pSalida.y
                    pRutaLibre.setdefault(0,(pVecino.x,pVecino.y))

                contId=1
                while(True):
                    pNodo=pVecino
                    n_indices=sorted(pSaved.keys())
                    if(len(n_indices)<=1):
                        break
                    for j in n_indices:
                        if(pNodo.x==pSaved[j][0] and pNodo.y==pSaved[j][1] ):
                            pSaved.pop(j)
                        
                    if(pasar==0):
                        pVecino=AlgoritmoPRM.pNeastRutaFinal(pNodo,pSaved)
                        if(not AlgoritmoPRM.ColisionaParedes(pNodo,pSalida,pObst) and AlgoritmoPRM.distanciaEuclideana(pNodo,pSalida)<4.5):
                            pRutaLibre.setdefault(contId,(pNodo.x,pNodo.y))
                            pRutaLibre.setdefault(contId+1,(pSalida.x,pSalida.y))
                            break
                        elif(not AlgoritmoPRM.ColisionaParedes(pVecino,pNodo,pObst)):
                            pRutaLibre.setdefault(contId,(pVecino.x,pVecino.y))
                            contId+=1
                        else:
                            #Elimino el nodo vecino que es lejano
                            if(contId>=2):
                                n_indices=sorted(pSaved.keys())
                                for k in n_indices:
                                    if(pVecino.x==pSaved[k][0] and pVecino.y==pSaved[k][1] ):
                                        pSaved.pop(k)
                                # vuelvo asignar el pNodo
                                pVecino.x=pRutaLibre[contId-2][0];pVecino.y=pRutaLibre[contId-2][1]
                    if(pasar==1):
                        pVecino=AlgoritmoPRM.pNeastRutaFinal(pNodo,pSaved)
                        if(not AlgoritmoPRM.ColisionaParedes(pNodo,pLlegada,pObst) and AlgoritmoPRM.distanciaEuclideana(pNodo,pLlegada)<3):
                            pRutaLibre.setdefault(contId,(pNodo.x,pNodo.y))
                            pRutaLibre.setdefault(contId+1,(pLlegada.x,pLlegada.y))
                            break
                        elif(not AlgoritmoPRM.ColisionaParedes(pVecino,pNodo,pObst)):
                            pRutaLibre.setdefault(contId,(pVecino.x,pVecino.y))
                            contId+=1
                        else:
                            #Elimino el nodo vecino que es lejano
                            if(contId>=2):
                                n_indices=sorted(pSaved.keys())
                                for k in n_indices:
                                    if(pVecino.x==pSaved[k][0] and pVecino.y==pSaved[k][1] ):
                                        pSaved.pop(k)
                                pVecino.x=pRutaLibre[contId-2][0];pVecino.y=pRutaLibre[contId-2][1]

                pasar+=1
                pSaved.clear()
                for i in range(0,len(pRutaLibre)):
                    pSaved.setdefault(i,(pRutaLibre[i][0],pRutaLibre[i][1]))

                #Grafico la ruta libre en RVIZ 
            sumaRuta=0
            for i in range(1,len(pRutaLibre)):
                rospy.sleep(0.2)
                p1.x=pRutaLibre[i-1][0];p1.y=pRutaLibre[i-1][1]
                p2.x=pRutaLibre[i][0];p2.y=pRutaLibre[i][1]
                sumaRuta=sumaRuta+AlgoritmoPRM.distanciaEuclideana(p1,p2)
                Marcadores.RutaLibre(p1,p2,pub_markers,i)
                tEnd=rospy.Time.now().to_sec()-tInit
                print("Dist Tramo",i,":",AlgoritmoPRM.distanciaEuclideana(p1,p2),"\tTiempo: ",tEnd)
            tEnd=rospy.Time.now().to_sec()-tInit
            print("--------------------------------------")
            print("Dist Ruta:",sumaRuta,"\tTiempo Ejecucion: ",tEnd)  
            break

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