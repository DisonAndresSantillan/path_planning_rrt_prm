#!/usr/bin/python3

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

pLlegada.x=-3;pLlegada.y=0.5
N=1000
sigma=0.8
tolRuta=0.95#0.85
espacioRutaParedes=0.3

class AlgoritmoRRT(object):

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
    def pRand():
        """
        Entrada:
            Configuración x,y maximos del mapa
        Salida:
            Punto random dentro del mapa
        """

        p=Point()
        p.x=random.uniform(xMin, xMax)
        p.y=random.uniform(yMin, yMax)

        return p

    
    #Determino el punto mas cercano 
    def pNeast(pRandom,pSaved):
        """
        Entrada:
            pRandom: punto random
            pSaved: son los puntos guardados que anteriormente fueron random
        Salida:
            puntos (x,y) mas cercano 
        """

        p=Point()
        n_puntos=len(pSaved)
        if(n_puntos==1):
            #transformo un dict a una tupla 
            p.x=pSaved[0][0];p.y=pSaved[0][1]
            return p
        # vacio todos los datos para volver a ingresar distancias
        distanciasPuntos={}
        distanciasPuntos.clear()
        for i in range(n_puntos):
            #Accedo a los datos dentro del diccionario
            p.x=pSaved[i][0];p.y=pSaved[i][1]
            d=AlgoritmoRRT.distanciaEuclideana(pRandom,p)
            #A continuacion ordeno el dict conforme la distancia mas cerca
            distanciasPuntos.setdefault(d,(pSaved[i][0],pSaved[i][1]))
            #obtengo la ubicacion de los puntos conforme la menor distancia
            # el dict en este caso guarla las distancias de mayor a menor
            distan=sorted(distanciasPuntos.keys())
            #print(" la distancia de las llaves es: ",distan[0])
            #print("i: ",i,"px: ",pSaved[i][0],"py: ",pSaved[i][1])
            #print("i: ",i,"\t P rand x: ",pRandom.x,"\t P rand y: ",pRandom.y,"\t P saved x: ",pSaved[i][0],"\t P saved y: ",pSaved[i][1],"distancia: ",AlgoritmoRRT.distanciaEuclideana(p,pRandom))

            #print("p_rand: (",p_rand.x,",",p_rand.y,")","puntos_gardados: ",puntos[i],"distancia: ",d)
            #Obtengo el primer valor del dict con base a la llave
        #print(distanciasPuntos)
        p.x=distanciasPuntos[distan[0]][0]
        p.y=distanciasPuntos[distan[0]][1]
        #print("-----------------")
        #print("p1 x: ",p.x,"\tp1 y: ",p.y)
        #print("------------------")
        return p



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
                d=AlgoritmoRRT.distanciaEuclideana(pRandom,p)
                #A continuacion ordeno el dict conforme la distancia mas cerca
                distanciasPuntos.setdefault(d,(pSaved[i][0],pSaved[i][1]))
                #obtengo la ubicacion de los puntos conforme la menor distancia
            distan=sorted(distanciasPuntos.keys())
            #print(distan)
            p.x=distanciasPuntos[distan[0]][0]
            p.y=distanciasPuntos[distan[0]][1]
        return p




    def pSteer(p1,p2,sigma,pObst):
        pNuevo=Point()

        if(p1.x !=p2.x):
            m=(p2.y-p1.y)/(p2.x-p1.x)
            theta=math.atan(m)

        if(theta<0):
            if((p2.x-p1.x)<0):
                theta=theta+math.pi
            else:
                theta=theta+2*math.pi
        else:
            if((p2.y-p1.y)<0 and (p2.x-p1.x)<0):
                theta=theta+math.pi
        pNuevo.x=sigma*math.cos(theta)+p1.x
        pNuevo.y=sigma*math.sin(theta)+p1.y
        pNuevo.z=0

        #RUTA LIBRE DE COLISIONES
        #calculo de interseccion con obstaculos
        if(not AlgoritmoRRT.ColisionaParedes(p1,pNuevo,pObst) and AlgoritmoRRT.fueraMapa(pNuevo)):
            
            return pNuevo

        return p1


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
            izq=(paredes.pose.position.x-paredes.scale.x/2)-espacioRutaParedes
            der=(paredes.pose.position.x+paredes.scale.x/2)+espacioRutaParedes
            abajo=(paredes.pose.position.y-paredes.scale.y/2)-espacioRutaParedes
            arriba=(paredes.pose.position.y+paredes.scale.y/2)+espacioRutaParedes

            #Comprobar si hay alguna interseccion

            if( AlgoritmoRRT.intersecarLineas(p1,p2,izq,abajo,der,abajo) or AlgoritmoRRT.intersecarLineas(p1,p2,izq,abajo,izq,arriba) or AlgoritmoRRT.intersecarLineas(p1,p2,der,abajo,der,arriba) or AlgoritmoRRT.intersecarLineas(p1,p2,izq,arriba,der,arriba)):
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
    def puntosNuevos(p1,cont,pub_markers):
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
        pNuevos.pose.orientation.w = 1.0
        pNuevos.scale.x = 0.5
        pNuevos.scale.y = 0.5
        pNuevos.scale.z = 0.0
        # marker color
        pNuevos.color.a = 1
        pNuevos.color.r = 255/255
        pNuevos.color.g = 0/255
        pNuevos.color.b = 0/255
        pNuevos.scale.x = 0.05
        pNuevos.scale.y = 0.05


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



    def Rutas(p1,p2,pub_makers,cont_id):
        unirPuntos=Marker();vertices=Marker()

        vertices.type=vertices.POINTS
        unirPuntos.type=unirPuntos.LINE_LIST
        unirPuntos.header.frame_id="map"
        unirPuntos.header.stamp=rospy.Time.now()
        unirPuntos.ns="rutasRRT"
        unirPuntos.id=3+cont_id
        unirPuntos.action=unirPuntos.ADD
        unirPuntos.pose.orientation.w=5
        unirPuntos.scale.x=0.02


        unirPuntos.color.r=130/255
        unirPuntos.color.g=0/255
        unirPuntos.color.b=0/255
        
        unirPuntos.color.a=1

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
    rospy.init_node('Algoritmo_RRT_ROBOT1')
    pub_markers = rospy.Publisher('Markers_R1', Marker, queue_size=10)
    rospy.Subscriber('/r1/odom',Odometry,r1PosXY)
    rospy.Rate(20)
    m=Marker()
    m.ns="rutasRRT"
    m.action=m.DELETEALL
    pub_markers.publish(m)
    runRRT(N,sigma,pSalida,pLlegada,pub_markers,pSaved)



def r1PosXY(msg):
    
    posRobot.x=msg.pose.pose.position.x
    posRobot.y=msg.pose.pose.position.y
    posRobot.z=0
    rot = msg.pose.pose.orientation
    (roll, pitch, theta_r1) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])




def runRRT(N,sigma,pSalida,pLlegada,pub_markers,pSaved):
    pRandom=Point();pNear=Point();pNew=Point();p1=Point();p2=Point()
    rutaEncontrada=False
    cont=0
    #inicializando la posicion del robot
    rospy.loginfo('Obteniendo la posicion del robot')
    rospy.sleep(1)
    pSalida.x=posRobot.x; pSalida.y=posRobot.y
    pSaved.setdefault(0,(pSalida.x,pSalida.y))
    #INICIO EL ALGORITMO RRT
    tInit=rospy.Time.now().to_sec()
    for i in range(1,N):

        rospy.sleep(0.01)
        #defino obstaculos en el entorno de RVIZ
        Marcadores.Meta(pSalida,pLlegada,pub_markers)
        Marcadores.Obstaculos(pub_markers)
        
        if(not rutaEncontrada):
            pRandom=AlgoritmoRRT.pRand()
            # puntos cercanos
            pNear=AlgoritmoRRT.pNeast(pRandom,pSaved)
            # puntos Nuevos
            pR=random.uniform(0,1)
            if(pR>tolRuta):
                pNew=AlgoritmoRRT.pSteer(pNear,pLlegada,sigma,pObst)

            else:
                pNew=AlgoritmoRRT.pSteer(pNear,pRandom,sigma,pObst)
           
            Marcadores.puntosNuevos(pNew,i,pub_markers)

            #Grafico las rutas
            Marcadores.Rutas(pNear,pNew,pub_markers,i)


            if(AlgoritmoRRT.distanciaEuclideana(pNew,pLlegada)<=1.5 and not AlgoritmoRRT.ColisionaParedes(pNew,pLlegada,pObst)):
                pNew.x=pLlegada.x;pNew.y=pLlegada.y
                pSaved.setdefault(cont+1,(pNew.x,pNew.y))
                Marcadores.Rutas(pNear,pNew,pub_markers,i+1)
                rutaEncontrada=True

            if((pNew.x != pNear.x) and (pNew.y != pNear.y)):
                cont+=1
                tPaso=rospy.Time.now().to_sec()-tInit
                print("it: ",i,"\t#nodos: ",cont,"\ttiempo: ",tPaso)
                pSaved.setdefault(cont,(pNew.x,pNew.y))

        #******************************************************************************************************************
        
        if(rutaEncontrada):
            """
                En esta funcion analizo todos los puntos y 
                busco una ruta para que siga el robot
                pSaved: son los puntos guardados 

            """
            #Defino variables
            #*****************************************************************************************
            pVecino=Point();pNodo=Point()
            pasar=0

            while(pasar<=1):
                pRutaLibre.clear()
                X=np.zeros((len(pSaved),2))
                for i in range(len(pSaved)):
                    X[i]=(pSaved[i][0],pSaved[i][1])
                #print(X)
                tree = KDTree(X, leaf_size=len(pSaved),metric="chebyshev")
                sentido=np.zeros((len(pSaved),2))
                x1=np.linspace(pSalida.x,pLlegada.x,len(X[:,1]))
                y1=np.linspace(pSalida.y,pLlegada.y,len(X[:,1]))
                for i in range(len(X[:,1])):
                    sentido[i]=(x1[i],y1[i])
                ind = tree.query_radius(sentido, r=40*sigma)   
                print("-------------------------------")
                print("Calculando Ruta Optima")
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
                        pVecino=AlgoritmoRRT.pNeastRutaFinal(pNodo,pSaved)
                        if(not AlgoritmoRRT.ColisionaParedes(pNodo,pSalida,pObst) and AlgoritmoRRT.distanciaEuclideana(pNodo,pSalida)<sigma*2):
                            pRutaLibre.setdefault(contId,(pNodo.x,pNodo.y))
                            pRutaLibre.setdefault(contId+1,(pSalida.x,pSalida.y))
                            break
                        elif(not AlgoritmoRRT.ColisionaParedes(pVecino,pNodo,pObst)):
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
                    if(pasar==1):
                        pVecino=AlgoritmoRRT.pNeastRutaFinal(pNodo,pSaved)
                        if(not AlgoritmoRRT.ColisionaParedes(pNodo,pLlegada,pObst) and AlgoritmoRRT.distanciaEuclideana(pNodo,pLlegada)<2):
                            pRutaLibre.setdefault(contId,(pNodo.x,pNodo.y))
                            pRutaLibre.setdefault(contId+1,(pLlegada.x,pLlegada.y))
                            break
                        elif(not AlgoritmoRRT.ColisionaParedes(pVecino,pNodo,pObst)):
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


                tEnd=rospy.Time.now().to_sec()-tInit
                print("-------------------------------\n RUTA FINAL \n Tiempo: ",tEnd)



                #Grafico la ruta libre en RVIZ 
            sumaRuta=0
            for i in range(1,len(pRutaLibre)):
                rospy.sleep(0.2)
                p1.x=pRutaLibre[i-1][0];p1.y=pRutaLibre[i-1][1]
                p2.x=pRutaLibre[i][0];p2.y=pRutaLibre[i][1]
                Marcadores.RutaLibre(p1,p2,pub_markers,i)
                tEnd=rospy.Time.now().to_sec()-tInit
                print("Dist Tramo",i,":",AlgoritmoRRT.distanciaEuclideana(p1,p2),"\tTiempo: ",tEnd)
            tEnd=rospy.Time.now().to_sec()-tInit
            print("--------------------------------------")
            print("Dist Ruta:",sumaRuta,"\tTiempo Ejecucion: ",tEnd)  
            break

if __name__=='__main__':
    try:
        s=1
        while(s!=0):
            main()
            pSaved.clear()
            pRutaLibre.clear()

            #Elimino los marcadores
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