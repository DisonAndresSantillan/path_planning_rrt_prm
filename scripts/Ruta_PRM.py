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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Vector3,Point
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan 
import Marcadores

#Variables Globales del ROBOT 1
global theta_rob;xMin=-6;xMax=6;yMin=-6;yMax=6
posRobot=Point();pSalida=Point(); pLlegada=Point()
pSaved={}
pRutaLibre={}
pObst=[]

pLlegada.x=-4.5;pLlegada.y=4;pSalida.x=3.7;pSalida.y=3.3
N=1000
global exAnt,eyAnt
exAnt=0;eyAnt=0


#Funcion principal
def main():
    global pub_markers
    tipoRobot=2
    prueba=1
    AlgoritmoPRM.runPRM(N,pLlegada,tipoRobot,prueba)


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





    def controlVelRobot(pMeta,t):
        """
        Entrada
            pMeta: punto a donde quiero llevar al robot
            pRobot: Localizacion del robot
        Salida
            V: velocidad lineal
            W: velocidad angular
        """
        global posRobot,theta_rob
        global y_l,y_r,s_d,distPared
        global exAnt,eyAnt
        vRobMax=0.422;wRobMax=2.84
        kp=0.5;kd=0.1;ki=0.1
        ex=0;ey=0;V=0;W=0;l=0.5
        
        #parametrizacion de la recta
        Xd=posRobot.x+(pMeta.x-posRobot.x)*t
        Yd=posRobot.y+(pMeta.y-posRobot.y)*t

        #Derivada de la parametrizacion
        Xdp=pMeta.x-posRobot.x
        Ydp=pMeta.y-posRobot.y

        pX=posRobot.x+l*math.cos(theta_rob)
        pY = posRobot.y+l*math.sin(theta_rob)
        
        #Calculo los errores
        ex=pX-Xd;ey=pY-Yd

        #Control cinematico
        #Ux = Xdp-kp*ex; Uy = Ydp-kp*ey
        Ux = Xdp-kp*(ex-exAnt); Uy = Ydp-kp*(ey-eyAnt)
        exAnt=ex;eyAnt=ey
        #Calculo de la velocidad acorde al modelo cinematico
        V = Ux*math.cos(theta_rob)+Uy*math.sin(theta_rob)
        W = -Ux*math.sin(theta_rob)/l+Uy*math.cos(theta_rob)/l

        #Saturaciones
        	#Velocities saturation
        if (abs(V)>vRobMax):
            V = vRobMax*abs(V)/V
        if (abs(W)>wRobMax):
            W = wRobMax*abs(W)/W

        #Evasion de obstaculos
        """
        delta = distPared-y_r 
        prev_error=0
        
        if(y_r<0.7):
            while(y_r<=0.5 and s_d<0.5):
                kp2 = 4
                kd2 = 200
                print(y_r)
                delta = distPared-y_r 
                PID_output  = kp2*delta + kd2*(delta-prev_error)
                prev_error      = delta
                W = np.clip(PID_output,-1.2,1.2)
                V= np.clip((s_d-0.35),-0.1,0.4)
                pub_msg = Twist(Vector3(V,0,0), Vector3(0,0,W))
                pub_vel.publish(pub_msg)
        elif(y_l<-1):
            while(y_l<=0.5 and s_d<0.5):
                kp2 = 4
                kd2 = 200
                print(y_r)
                delta = distPared-y_l
                PID_output  = kp2*delta + kd2*(delta-prev_error)
                prev_error      = delta
                W = np.clip(PID_output,-1.2,1.2)
                V= np.clip((s_d-0.35),-0.1,0.4)
                pub_msg = Twist(Vector3(V,0,0), Vector3(0,0,W))
                pub_vel.publish(pub_msg)

        """
        return V,W   





    def runPRM(N,pLlegada,tipoRobot,prueba):
        global pObst,posRobot,pub_markers,pub_vel
        global xMin,xMax,yMin,yMax

        if(tipoRobot==1):
            #inicialización ROBOT 1
            rospy.init_node('Algoritmo_PRM_ROBOT1')
            pub_markers = rospy.Publisher('mPRM_R1', Marker, queue_size=10)
            rospy.Subscriber('/r1/odom',Odometry,robPosXY)
            pub_vel=rospy.Publisher('/r1/cmd_vel',Twist,queue_size=10)
            rospy.Subscriber("/r1/scan", LaserScan, datSensorLaser)
            rospy.Rate(20)
        elif(tipoRobot==2):
                #inicialización ROBOT 1
            rospy.init_node('Algoritmo_PRM_ROBOT2')
            pub_markers = rospy.Publisher('mPRM_R2', Marker, queue_size=10)
            rospy.Subscriber('/r2/odom',Odometry,robPosXY)
            pub_vel=rospy.Publisher('/r2/cmd_vel',Twist,queue_size=10)
            rospy.Subscriber("/r2/scan", LaserScan,datSensorLaser)
            rospy.Rate(20)

        #Configuracion prueba 2
        if((prueba<=777 and prueba>=111)or (prueba==31 or prueba==32)):
            for i in range(15):
                pObst=Marcadores.Marcadores.Obstaculos(pub_markers,prueba)
                rospy.sleep(0.01)
            myt=input("\nPor favor configure la posición de salida del robot con los punteros (2D Nav Goal) de Rviz y posteriormente presione ENTER para continuar... ")  
            while(not AlgoritmoPRM.fueraMapa(pLlegada)):
                pObst=Marcadores.Marcadores.Obstaculos(pub_markers,prueba)
                pLlegada.x=float(input("Ingrese la coordenada X de Llegada: "))
                pLlegada.y=float(input("Ingrese la coordenada Y de Llegada: "))
                if(not AlgoritmoPRM.fueraMapa(pLlegada)):
                    print("Coordenada fuera del rango")
        

        if(prueba==31):
            xMin=-4;xMax=6;yMin=-3;yMax=6
        if(prueba==32):
            xMin=-4;xMax=6;yMin=-6;yMax=0
        #Limpio los marcadores
        m=Marker()
        m.ns="rutasPRM"
        m.action=m.DELETEALL
        pub_markers.publish(m)

        #Inicializo al robot
        vel_msg=Twist()
        vel_msg.linear.x=vel_msg.linear.y=vel_msg.linear.z=0
        vel_msg.angular.z=0
        pub_vel.publish(vel_msg)

        pRandom=Point();p1=Point();p2=Point()
        rutaEncontrada=False
        #inicializando la posicion del robot
        pSalida=posRobot
        pSaved.setdefault(0,(pSalida.x,pSalida.y))
        #INICIO EL ALGORITMO PRM
        tInit=rospy.Time.now().to_sec()
        for i in range(1,N):
            rospy.sleep(0.01)
            
            #defino obstaculos en el entorno de RVIZ
            Marcadores.Marcadores.Meta(pSalida,pLlegada,pub_markers)
            pObst=Marcadores.Marcadores.Obstaculos(pub_markers,prueba)
            if(not rutaEncontrada):
                pRandom=AlgoritmoPRM.pRand(pObst)
                Marcadores.Marcadores.puntosRandom(pRandom,i,pub_markers,tipoRobot)#----------------------------
                if(prueba==41 or prueba==42):
                    tPaso=rospy.Time.now().to_sec()-tInit
                    print("it: ",i,"\t#nodos: ",i,"\ttiempo: ",tPaso)
                    pSaved.setdefault(i,(pRandom.x,pRandom.y))
                    if(i==N-1):
                        pSaved.setdefault(i,(pLlegada.x,pLlegada.y))
                        rutaEncontrada=True
                else:
                    if(AlgoritmoPRM.distanciaEuclideana(pRandom,pLlegada)<=1 and not AlgoritmoPRM.ColisionaParedes(pRandom,pLlegada,pObst) and i>40):
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
                                Marcadores.Marcadores.Rutas(p1,p2,pub_markers,ij,tipoRobot)#-----------------
                                ij+=1
                        #*****************************************************************************************
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
                print("****************************************************************************************************")
                print("                                        CALCULANDO RUTA DE NAVEGACION   " )

                #Grafico la ruta libre en RVIZ 
                sumaRuta=0
                for i in range(1,len(pRutaLibre)):
                    rospy.sleep(0.2)
                    p1.x=pRutaLibre[i-1][0];p1.y=pRutaLibre[i-1][1]
                    p2.x=pRutaLibre[i][0];p2.y=pRutaLibre[i][1]
                    sumaRuta=sumaRuta+AlgoritmoPRM.distanciaEuclideana(p1,p2)
                    Marcadores.Marcadores.RutaLibre(p1,p2,pub_markers,i,tipoRobot)#-----------------------------
                    #tEnd=rospy.Time.now().to_sec()-tInit
                    #print("Dist Tramo",i,":",AlgoritmoPRM.distanciaEuclideana(p1,p2),"\tTiempo: ",tEnd)
                tEnd=rospy.Time.now().to_sec()-tInit
                
                print("\n     Distancia Ruta:",sumaRuta,"\t [px/m]","\tTiempo Ejecucion: ",tEnd,"\t[seg]\n")  
                print("                                        MOVIENDO EL ROBOT A LA RUTA")
                print("****************************************************************************************************")

                #Mover Robot a la Ruta
                global t,t0 
                t0=rospy.Time.now().to_sec()
                vel_msg=Twist()
                p1.x=0;p1.y=0;p2.x=0;p2.y=0
                for i in range(0,len(pRutaLibre)):
      
                    t = rospy.Time.now().to_sec()-t0
                    p1.x=posRobot.x;p1.y=posRobot.y
                    p2.x=pRutaLibre[i][0];p2.y=pRutaLibre[i][1]
                    while(AlgoritmoPRM.distanciaEuclideana(p1,p2)>=0.1):
                        p1.x=posRobot.x;p1.y=posRobot.y
                        [V,W]=AlgoritmoPRM.controlVelRobot(p2,t)
                        vel_msg.linear.x=V;vel_msg.angular.z=W                    
                        pub_vel.publish(vel_msg)

                vel_msg=Twist()
                vel_msg.linear.x=vel_msg.linear.y=vel_msg.linear.z=0
                vel_msg.angular.z=0
                pub_vel.publish(vel_msg)
                break







def robPosXY(msg):
    global theta_rob
    
    posRobot.x=msg.pose.pose.position.x
    posRobot.y=msg.pose.pose.position.y
    posRobot.z=0
    rot = msg.pose.pose.orientation
    (roll, pitch, theta_rob) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])



def datSensorLaser(data):
    global y_l,y_r,s_d,distPared
    side_scanstartangle = 20 
    side_scanrange      = 60          
    front_scanrange     = 20
    distPared    = 1.5
    x   = np.zeros((180))
    x = list(data.ranges)
    for i in range(180):
        if(x[i]==np.inf):
            x[i]=7
        elif(x[i]==0):
            x[i]=6
    # store scan data 
    y_l= min(x[side_scanstartangle:side_scanstartangle+side_scanrange])          # left wall distance
    y_r= min(x[180-side_scanstartangle-side_scanrange:180-side_scanstartangle])  # right wall distance
    s_d=min(min(x[0:int(front_scanrange/2)],x[int(180-front_scanrange/2):180])) # front wall distance


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