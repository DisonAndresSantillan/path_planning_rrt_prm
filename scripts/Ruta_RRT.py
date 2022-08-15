#!/usr/bin/python3

import rospy 
import random
import numpy as np
from sklearn.neighbors import KDTree
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist,Vector3,Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan 
import Marcadores

#Variables Globales del ROBOT 1
global theta_rob;xMin=-6;xMax=6;yMin=-6;yMax=6
posRobot=Point();pSalida=Point(); pLlegada=Point()
pSaved={}
pRutaLibre={}
pObst=[]
global exAnt,eyAnt
exAnt=0;eyAnt=0
pLlegada.x=-3;pLlegada.y=0.5
N=1000
sigma=0.8
tolRuta=0.9#0.1
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
        vRobMax=0.22;wRobMax=2.84
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

        return V,W   



    def runRRT(N,pLlegada,sigma,tipoRobot,prueba):
        global pObst,posRobot,pub_markers,pub_vel,pSaved,theta_rob

        if(tipoRobot==1):
            #inicialización ROBOT 1
            rospy.init_node('Algoritmo_RRT_ROBOT1')
            pub_markers = rospy.Publisher('mRRT_R1', Marker, queue_size=10)
            rospy.Subscriber('/r1/odom',Odometry,robPosXY)
            pub_vel=rospy.Publisher('/r1/cmd_vel',Twist,queue_size=10)
            rospy.Subscriber("/r1/scan", LaserScan, datSensorLaser)
            rospy.Rate(40)
        elif(tipoRobot==2):
                #inicialización ROBOT 1
            rospy.init_node('Algoritmo_RRT_ROBOT2')
            pub_markers = rospy.Publisher('mRRT_R2', Marker, queue_size=10)
            rospy.Subscriber('/r2/odom',Odometry,robPosXY)
            pub_vel=rospy.Publisher('/r2/cmd_vel',Twist,queue_size=10)
            rospy.Subscriber("/r2/scan", LaserScan,datSensorLaser)
            rospy.Rate(40)
        #Limpio los marcadores
        m=Marker()
        m.ns="rutasRRT"
        m.action=m.DELETEALL
        pub_markers.publish(m)

        #Inicializo al robot
        vel_msg=Twist()
        vel_msg.linear.x=vel_msg.linear.y=vel_msg.linear.z=0
        vel_msg.angular.z=0
        pub_vel.publish(vel_msg)

        pSaved.clear()
        pRutaLibre.clear()

        #Configuracion prueba 2 y 3
        if((prueba<=777 and prueba>=111) or prueba==3):
            for i in range(15):
                pObst=Marcadores.Marcadores.Obstaculos(pub_markers,prueba)
                rospy.sleep(0.01)
            muy=input("\nPor favor configure la posición de salida del robot con los punteros (2D Nav Goal) de Rviz y posteriormente presione ENTER para continuar... ")  
            while(not AlgoritmoRRT.fueraMapa(pLlegada)):
                pObst=Marcadores.Marcadores.Obstaculos(pub_markers,prueba)
                pLlegada.x=float(input("Ingrese la coordenada X de Llegada: "))
                pLlegada.y=float(input("Ingrese la coordenada Y de Llegada: "))
                if(not AlgoritmoRRT.fueraMapa(pLlegada)):
                    print("Coordenada fuera del rango")

        
        pRandom=Point();pNear=Point();pNew=Point();p1=Point();p2=Point()
        rutaEncontrada=False
        cont=0
        #inicializando la posicion del robot
        rospy.sleep(0.5)
        pSalida.x=posRobot.x; pSalida.y=posRobot.y
        pSaved.setdefault(0,(pSalida.x,pSalida.y))
        #INICIO EL ALGORITMO RRT
        tInit=rospy.Time.now().to_sec()
        for i in range(1,N):

            rospy.sleep(0.01)
            #defino obstaculos en el entorno de RVIZ
            Marcadores.Marcadores.Meta(pSalida,pLlegada,pub_markers)
            pObst=Marcadores.Marcadores.Obstaculos(pub_markers,prueba)
            
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
            
                Marcadores.Marcadores.puntosRandom(pNew,i,pub_markers,tipoRobot)

                #Grafico las rutas
                Marcadores.Marcadores.Rutas(pNear,pNew,pub_markers,i,tipoRobot)


                if(AlgoritmoRRT.distanciaEuclideana(pNew,pLlegada)<=1.5 and not AlgoritmoRRT.ColisionaParedes(pNew,pLlegada,pObst)):
                    pSaved.setdefault(cont+1,(pNew.x,pNew.y))
                    Marcadores.Marcadores.Rutas(pNear,pNew,pub_markers,i+1,tipoRobot)
                    pNew.x=pLlegada.x;pNew.y=pLlegada.y
                    pSaved.setdefault(cont+2,(pNew.x,pNew.y))
                    Marcadores.Marcadores.Rutas(pNear,pNew,pub_markers,i+2,tipoRobot)
                    rutaEncontrada=True

                if((pNew.x != pNear.x) and (pNew.y != pNear.y)):
                    cont+=1
                    tPaso=rospy.Time.now().to_sec()-tInit
                    print("iteraciones: ",i,"\t#nodos: ",cont,"\ttiempo: ",tPaso ,"\t seg")
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
                    ind = tree.query_radius(sentido, r=40*(sigma))   
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
                            if(not AlgoritmoRRT.ColisionaParedes(pNodo,pSalida,pObst) and AlgoritmoRRT.distanciaEuclideana(pNodo,pSalida)<1):
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


                
                print("****************************************************************************************************")
                print("                                      CALCULANDO RUTA DE NAVEGACION   " )

                #Grafico la ruta libre en RVIZ 
                sumaRuta=0
                for i in range(1,len(pRutaLibre)):
                    rospy.sleep(0.2)
                    p1.x=pRutaLibre[i-1][0];p1.y=pRutaLibre[i-1][1]
                    p2.x=pRutaLibre[i][0];p2.y=pRutaLibre[i][1]
                    sumaRuta=sumaRuta+AlgoritmoRRT.distanciaEuclideana(p1,p2)
                    Marcadores.Marcadores.RutaLibre(p1,p2,pub_markers,i,tipoRobot)#-----------------------------
                    #tEnd=rospy.Time.now().to_sec()-tInit
                    #print("Dist Tramo",i,":",AlgoritmoRRT.distanciaEuclideana(p1,p2),"\tTiempo: ",tEnd)
                tEnd=rospy.Time.now().to_sec()-tInit
                
                print("\n  Distancia Ruta:",sumaRuta,"\t [px/m]","\tTiempo Ejecucion: ",tEnd,"\t[seg]\n")   
                print("                                      MOVIENDO EL ROBOT A LA RUTA")
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
                    while(AlgoritmoRRT.distanciaEuclideana(p1,p2)>=0.1):
                        p1.x=posRobot.x;p1.y=posRobot.y
                        [V,W]=AlgoritmoRRT.controlVelRobot(p2,t)
                        vel_msg.linear.x=V;vel_msg.angular.z=W                    
                        pub_vel.publish(vel_msg)
                vel_msg=Twist()
                vel_msg.linear.x=vel_msg.linear.y=vel_msg.linear.z=0
                vel_msg.angular.z=0
                pub_vel.publish(vel_msg)

                break






#Funcion principal
def main():
    global pub_markers
    #inicialización
    tipoRobot=2
    prueba=1
    N=1000
    #sigma=0.8
    AlgoritmoRRT.runRRT(N,pLlegada,sigma,tipoRobot,prueba)



def robPosXY(msg):
    global posRobot,theta_rob
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