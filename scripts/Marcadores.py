import rospy 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random
pObst=[]

class Marcadores(object):

    # grafico en RVIZ los puntos random
    def puntosRandom(p1,cont,pub_markers,tipoRob):
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
        if(tipoRob==1):
            pNuevos.color.r = 255/255
            pNuevos.color.g = 0/255
            pNuevos.color.b = 0/255
        elif(tipoRob==2):
            pNuevos.color.r = 125/255
            pNuevos.color.g = 224/255
            pNuevos.color.b = 38/255

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





    def Obstaculos(pub_markers,prueba):
        global pObst
        pared1=Marker(); pared2=Marker(); pared3=Marker(); pared4=Marker(); pared5=Marker(); pared6=Marker()
        pared7=Marker(); pared8=Marker(); pared9=Marker(); pared9=Marker(); pared10=Marker()
        pared11=Marker(); pared12=Marker(); pared13=Marker()
        obstPared=Marker();obstPared1=Marker();obstPared2=Marker();obstPared3=Marker();obstPared4=Marker()
        obstPared5=Marker();obstPared6=Marker();obstPared7=Marker();obstPared8=Marker();obstPared9=Marker()
        obstPared8=Marker();obstPared9=Marker();obstPared10=Marker();obstPared11=Marker();obstPared12=Marker();obstPared13=Marker()
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
        obstPared.id = 13
        obstPared.ns ="Obstaculos"
        obstPared.lifetime=rospy.Duration()
        obstPared.action = obstPared.ADD
        obstPared.scale.x = 1.95
        obstPared.scale.y = 0.45
        obstPared.scale.z = 0.5
        obstPared.pose.position.x = -0.625
        obstPared.pose.position.y = -2.725
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
        obstPared1.id = 14
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
        obstPared1.color.a = 15
        obstPared1.color.r = 255/255
        obstPared1.color.g = 200/255
        obstPared1.color.b = 0/255

        obstPared2.type =obstPared2.CUBE
        obstPared2.header.frame_id="map"
        obstPared2.id = 16
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

        obstPared3.type =obstPared3.CUBE
        obstPared3.header.frame_id="map"
        obstPared3.id = 17
        obstPared3.ns ="Obstaculos"
        obstPared3.lifetime=rospy.Duration()
        obstPared3.action = obstPared3.ADD
        obstPared3.scale.x = 0.3
        obstPared3.scale.y = 1.6
        obstPared3.scale.z = 0.5
        obstPared3.pose.position.x = -1.75
        obstPared3.pose.position.y = -0.35
        obstPared3.pose.position.z = 0.25
        obstPared3.pose.orientation.x = 0.0
        obstPared3.pose.orientation.y = 0.0
        obstPared3.pose.orientation.z = 0
        obstPared3.pose.orientation.w = 1
        obstPared3.color.a = 1
        obstPared3.color.r = 255/255
        obstPared3.color.g = 200/255
        obstPared3.color.b = 0/255


        obstPared4.type =obstPared4.CUBE
        obstPared4.header.frame_id="map"
        obstPared4.id = 18
        obstPared4.ns ="Obstaculos"
        obstPared4.lifetime=rospy.Duration()
        obstPared4.action = obstPared4.ADD
        obstPared4.scale.x = 0.3
        obstPared4.scale.y = 1.6
        obstPared4.scale.z = 0.5
        obstPared4.pose.position.x = 1.8
        obstPared4.pose.position.y = -1.7
        obstPared4.pose.position.z = 0.25
        obstPared4.pose.orientation.x = 0.0
        obstPared4.pose.orientation.y = 0.0
        obstPared4.pose.orientation.z = 0
        obstPared4.pose.orientation.w = 1
        obstPared4.color.a = 1
        obstPared4.color.r = 255/255
        obstPared4.color.g = 200/255
        obstPared4.color.b = 0/255


        obstPared5.type =obstPared5.CUBE
        obstPared5.header.frame_id="map"
        obstPared5.id = 19
        obstPared5.ns ="Obstaculos"
        obstPared5.lifetime=rospy.Duration()
        obstPared5.action = obstPared5.ADD
        obstPared5.scale.x = 0.3
        obstPared5.scale.y = 1.6
        obstPared5.scale.z = 0.5
        obstPared5.pose.position.x = -3.75
        obstPared5.pose.position.y = 1.05
        obstPared5.pose.position.z = 0.25
        obstPared5.pose.orientation.x = 0.0
        obstPared5.pose.orientation.y = 0.0
        obstPared5.pose.orientation.z = 0
        obstPared5.pose.orientation.w = 1
        obstPared5.color.a = 1
        obstPared5.color.r = 255/255
        obstPared5.color.g = 200/255
        obstPared5.color.b = 0/255

        obstPared6.type =obstPared6.CUBE
        obstPared6.header.frame_id="map"
        obstPared6.id = 20
        obstPared6.ns ="Obstaculos"
        obstPared6.lifetime=rospy.Duration()
        obstPared6.action =  obstPared6.ADD
        obstPared6.scale.x = 0.3
        obstPared6.scale.y = 2.4
        obstPared6.scale.z = 0.5
        obstPared6.pose.position.x = 4.45
        obstPared6.pose.position.y = 2.9
        obstPared6.pose.position.z = 0.25
        obstPared6.pose.orientation.x = 0.0
        obstPared6.pose.orientation.y = 0.0
        obstPared6.pose.orientation.z = 0
        obstPared6.pose.orientation.w = 1
        obstPared6.color.a = 1
        obstPared6.color.r = 255/255
        obstPared6.color.g = 200/255
        obstPared6.color.b = 0/255

        obstPared7.type =obstPared7.CUBE
        obstPared7.header.frame_id="map"
        obstPared7.id = 21
        obstPared7.ns ="Obstaculos"
        obstPared7.lifetime=rospy.Duration()
        obstPared7.action =   obstPared7.ADD
        obstPared7.scale.x = 4.35
        obstPared7.scale.y = 0.3
        obstPared7.scale.z = 0.5
        obstPared7.pose.position.x = 2.43
        obstPared7.pose.position.y = 4
        obstPared7.pose.position.z = 0.25
        obstPared7.pose.orientation.x = 0.0
        obstPared7.pose.orientation.y = 0.0
        obstPared7.pose.orientation.z = 0
        obstPared7.pose.orientation.w = 1
        obstPared7.color.a = 1
        obstPared7.color.r = 255/255
        obstPared7.color.g = 200/255
        obstPared7.color.b = 0/255

        obstPared8.type =obstPared8.CUBE
        obstPared8.header.frame_id="map"
        obstPared8.id = 22
        obstPared8.ns ="Obstaculos"
        obstPared8.lifetime=rospy.Duration()
        obstPared8.action =   obstPared8.ADD
        obstPared8.scale.x = 0.3
        obstPared8.scale.y = 11.58
        obstPared8.scale.z = 0.5
        obstPared8.pose.position.x = 5.7
        obstPared8.pose.position.y = -0.15
        obstPared8.pose.position.z = 0.25
        obstPared8.pose.orientation.x = 0.0
        obstPared8.pose.orientation.y = 0.0
        obstPared8.pose.orientation.z = 0
        obstPared8.pose.orientation.w = 1
        obstPared8.color.a = 1
        obstPared8.color.r = 255/255
        obstPared8.color.g = 200/255
        obstPared8.color.b = 0/255

        obstPared9.type =obstPared9.CUBE
        obstPared9.header.frame_id="map"
        obstPared9.id = 23
        obstPared9.ns ="Obstaculos"
        obstPared9.lifetime=rospy.Duration()
        obstPared9.action =   obstPared9.ADD
        obstPared9.scale.x = 0.3
        obstPared9.scale.y = 3
        obstPared9.scale.z = 0.5
        obstPared9.pose.position.x = 3.5
        obstPared9.pose.position.y = -1.1
        obstPared9.pose.position.z = 0.25
        obstPared9.pose.orientation.x = 0.0
        obstPared9.pose.orientation.y = 0.0
        obstPared9.pose.orientation.z = 0
        obstPared9.pose.orientation.w = 1
        obstPared9.color.a = 1
        obstPared9.color.r = 255/255
        obstPared9.color.g = 200/255
        obstPared9.color.b = 0/255

        obstPared10.type =obstPared10.CUBE
        obstPared10.header.frame_id="map"
        obstPared10.id = 24
        obstPared10.ns ="Obstaculos"
        obstPared10.lifetime=rospy.Duration()
        obstPared10.action =   obstPared10.ADD
        obstPared10.scale.x = 2.1
        obstPared10.scale.y = 0.3
        obstPared10.scale.z = 0.5
        obstPared10.pose.position.x = -0.55
        obstPared10.pose.position.y = -1.3
        obstPared10.pose.position.z = 0.25
        obstPared10.pose.orientation.x = 0.0
        obstPared10.pose.orientation.y = 0.0
        obstPared10.pose.orientation.z = 0
        obstPared10.pose.orientation.w = 1
        obstPared10.color.a = 1
        obstPared10.color.r = 255/255
        obstPared10.color.g = 200/255
        obstPared10.color.b = 0/255

        obstPared11.type =obstPared11.CUBE
        obstPared11.header.frame_id="map"
        obstPared11.id = 25
        obstPared11.ns ="Obstaculos"
        obstPared11.lifetime=rospy.Duration()
        obstPared11.action =   obstPared11.ADD
        obstPared11.scale.x = 0.3
        obstPared11.scale.y = 1.35
        obstPared11.scale.z = 0.5
        obstPared11.pose.position.x = 0.5
        obstPared11.pose.position.y = -1.825
        obstPared11.pose.position.z = 0.25
        obstPared11.pose.orientation.x = 0.0
        obstPared11.pose.orientation.y = 0.0
        obstPared11.pose.orientation.z = 0
        obstPared11.pose.orientation.w = 1
        obstPared11.color.a = 1
        obstPared11.color.r = 255/255
        obstPared11.color.g = 200/255
        obstPared11.color.b = 0/255

        obstPared12.type =obstPared12.CUBE
        obstPared12.header.frame_id="map"
        obstPared12.id = 26
        obstPared12.ns ="Obstaculos"
        obstPared12.lifetime=rospy.Duration()
        obstPared12.action =   obstPared12.ADD
        obstPared12.scale.x = 0.75
        obstPared12.scale.y = 0.3
        obstPared12.scale.z = 0.5
        obstPared12.pose.position.x = 3.275
        obstPared12.pose.position.y = -2.65
        obstPared12.pose.position.z = 0.25
        obstPared12.pose.orientation.x = 0.0
        obstPared12.pose.orientation.y = 0.0
        obstPared12.pose.orientation.z = 0
        obstPared12.pose.orientation.w = 1
        obstPared12.color.a = 1
        obstPared12.color.r = 255/255
        obstPared12.color.g = 200/255
        obstPared12.color.b = 0/255

        obstPared13.type =obstPared13.CUBE
        obstPared13.header.frame_id="map"
        obstPared13.id = 27
        obstPared13.ns ="Obstaculos"
        obstPared13.lifetime=rospy.Duration()
        obstPared13.action =   obstPared13.ADD
        obstPared13.scale.x = 2.5
        obstPared13.scale.y = 0.3
        obstPared13.scale.z = 0.5
        obstPared13.pose.position.x = 4.48+0.125
        obstPared13.pose.position.y = 0.25
        obstPared13.pose.position.z = 0.25
        obstPared13.pose.orientation.x = 0.0
        obstPared13.pose.orientation.y = 0.0
        obstPared13.pose.orientation.z = 0
        obstPared13.pose.orientation.w = 1
        obstPared13.color.a = 1
        obstPared13.color.r = 255/255
        obstPared13.color.g = 200/255
        obstPared13.color.b = 0/255

        obstPared14=Marker()
        obstPared14.type =obstPared14.CUBE
        obstPared14.header.frame_id="map"
        obstPared14.id = 28
        obstPared14.ns ="Obstaculos"
        obstPared14.lifetime=rospy.Duration()
        obstPared14.action =   obstPared14.ADD
        obstPared14.scale.x = 9.5
        obstPared14.scale.y = 0.3
        obstPared14.scale.z = 0.5
        obstPared14.pose.position.x = 1
        obstPared14.pose.position.y = 5.5
        obstPared14.pose.position.z = 0.25
        obstPared14.pose.orientation.x = 0.0
        obstPared14.pose.orientation.y = 0.0
        obstPared14.pose.orientation.z = 0
        obstPared14.pose.orientation.w = 1
        obstPared14.color.a = 1
        obstPared14.color.r = 255/255
        obstPared14.color.g = 200/255
        obstPared14.color.b = 0/255
        
        obstPared15=Marker()
        obstPared15.type =obstPared15.CUBE
        obstPared15.header.frame_id="map"
        obstPared15.id = 29
        obstPared15.ns ="Obstaculos"
        obstPared15.lifetime=rospy.Duration()
        obstPared15.action =   obstPared15.ADD
        obstPared15.scale.x = 9.5
        obstPared15.scale.y = 0.3
        obstPared15.scale.z = 0.5
        obstPared15.pose.position.x = 1
        obstPared15.pose.position.y = -5.8
        obstPared15.pose.position.z = 0.25
        obstPared15.pose.orientation.x = 0.0
        obstPared15.pose.orientation.y = 0.0
        obstPared15.pose.orientation.z = 0
        obstPared15.pose.orientation.w = 1
        obstPared15.color.a = 1
        obstPared15.color.r = 255/255
        obstPared15.color.g = 200/255
        obstPared15.color.b = 0/255

        obstPared16=Marker()
        obstPared16.type =obstPared16.CUBE
        obstPared16.header.frame_id="map"
        obstPared16.id = 30
        obstPared16.ns ="Obstaculos"
        obstPared16.lifetime=rospy.Duration()
        obstPared16.action =   obstPared16.ADD
        obstPared16.scale.x = 0.3
        obstPared16.scale.y = 3.2
        obstPared16.scale.z = 0.5
        obstPared16.pose.position.x = -3.75
        obstPared16.pose.position.y = -4.35
        obstPared16.pose.position.z = 0.25
        obstPared16.pose.orientation.x = 0.0
        obstPared16.pose.orientation.y = 0.0
        obstPared16.pose.orientation.z = 0
        obstPared16.pose.orientation.w = 1
        obstPared16.color.a = 1
        obstPared16.color.r = 255/255
        obstPared16.color.g = 200/255
        obstPared16.color.b = 0/255


        obstPared17=Marker()
        obstPared17.type =obstPared17.CUBE
        obstPared17.header.frame_id="map"
        obstPared17.id = 31
        obstPared17.ns ="Obstaculos"
        obstPared17.lifetime=rospy.Duration()
        obstPared17.action =   obstPared17.ADD
        obstPared17.scale.x = 0.3
        obstPared17.scale.y = 1.5
        obstPared17.scale.z = 0.5
        obstPared17.pose.position.x = -3.75
        obstPared17.pose.position.y = 4.9
        obstPared17.pose.position.z = 0.25
        obstPared17.pose.orientation.x = 0.0
        obstPared17.pose.orientation.y = 0.0
        obstPared17.pose.orientation.z = 0
        obstPared17.pose.orientation.w = 1
        obstPared17.color.a = 1
        obstPared17.color.r = 255/255
        obstPared17.color.g = 200/255
        obstPared17.color.b = 0/255

        obstPared18=Marker()
        obstPared18.type =obstPared18.CUBE
        obstPared18.header.frame_id="map"
        obstPared18.id = 32
        obstPared18.ns ="Obstaculos"
        obstPared18.lifetime=rospy.Duration()
        obstPared18.action =   obstPared18.ADD
        obstPared18.scale.x = 1.3
        obstPared18.scale.y = 1.8
        obstPared18.scale.z = 0.5
        obstPared18.pose.position.x = 4.8+0.2#4.6+0.25
        obstPared18.pose.position.y = -4.8
        obstPared18.pose.position.z = 0.25
        obstPared18.pose.orientation.x = 0.0
        obstPared18.pose.orientation.y = 0.0
        obstPared18.pose.orientation.z = 0
        obstPared18.pose.orientation.w = 1
        obstPared18.color.a = 1
        obstPared18.color.r = 255/255
        obstPared18.color.g = 200/255
        obstPared18.color.b = 0/255

        obstPared19=Marker()
        obstPared19.type =obstPared19.CUBE
        obstPared19.header.frame_id="map"
        obstPared19.id = 33
        obstPared19.ns ="Obstaculos"
        obstPared19.lifetime=rospy.Duration()
        obstPared19.action =   obstPared19.ADD
        obstPared19.scale.x = 0.3
        obstPared19.scale.y = 2
        obstPared19.scale.z = 0.5
        obstPared19.pose.position.x = 4.15
        obstPared19.pose.position.y = 2.9+0.1
        obstPared19.pose.position.z = 0.25
        obstPared19.pose.orientation.x = 0.0
        obstPared19.pose.orientation.y = 0.0
        obstPared19.pose.orientation.z = 0
        obstPared19.pose.orientation.w = 1
        obstPared19.color.a = 1
        obstPared19.color.r = 255/255
        obstPared19.color.g = 200/255
        obstPared19.color.b = 0/255

        obstPared20=Marker()
        obstPared20.type =obstPared20.CUBE
        obstPared20.header.frame_id="map"
        obstPared20.id = 34
        obstPared20.ns ="Obstaculos"
        obstPared20.lifetime=rospy.Duration()
        obstPared20.action =   obstPared20.ADD
        obstPared20.scale.x = 4.05
        obstPared20.scale.y = 0.3
        obstPared20.scale.z = 0.5
        obstPared20.pose.position.x = 2.275
        obstPared20.pose.position.y = 4
        obstPared20.pose.position.z = 0.25
        obstPared20.pose.orientation.x = 0.0
        obstPared20.pose.orientation.y = 0.0
        obstPared20.pose.orientation.z = 0
        obstPared20.pose.orientation.w = 1
        obstPared20.color.a = 1
        obstPared20.color.r = 255/255
        obstPared20.color.g = 200/255
        obstPared20.color.b = 0/255

        obstPared21=Marker()
        obstPared21.type =obstPared21.CUBE
        obstPared21.header.frame_id="map"
        obstPared21.id = 35
        obstPared21.ns ="Obstaculos"
        obstPared21.lifetime=rospy.Duration()
        obstPared21.action =   obstPared21.ADD
        obstPared21.scale.x = 2.1
        obstPared21.scale.y = 0.3
        obstPared21.scale.z = 0.5
        obstPared21.pose.position.x = 1.7
        obstPared21.pose.position.y = -4.5
        obstPared21.pose.position.z = 0.25
        obstPared21.pose.orientation.x = 0.0
        obstPared21.pose.orientation.y = 0.0
        obstPared21.pose.orientation.z = 0
        obstPared21.pose.orientation.w = 1
        obstPared21.color.a = 1
        obstPared21.color.r = 255/255
        obstPared21.color.g = 200/255
        obstPared21.color.b = 0/255

        obstPared22=Marker()
        obstPared22.type =obstPared22.CUBE
        obstPared22.header.frame_id="map"
        obstPared22.id = 36
        obstPared22.ns ="Obstaculos"
        obstPared22.lifetime=rospy.Duration()
        obstPared22.action =   obstPared22.ADD
        obstPared22.scale.x = 0.3
        obstPared22.scale.y = 1.05
        obstPared22.scale.z = 0.5
        obstPared22.pose.position.x = 2.75
        obstPared22.pose.position.y = -4.1-0.05/2
        obstPared22.pose.position.z = 0.25
        obstPared22.pose.orientation.x = 0.0
        obstPared22.pose.orientation.y = 0.0
        obstPared22.pose.orientation.z = 0
        obstPared22.pose.orientation.w = 1
        obstPared22.color.a = 1
        obstPared22.color.r = 255/255
        obstPared22.color.g = 200/255
        obstPared22.color.b = 0/255

        obstPared23=Marker()
        obstPared23.type =obstPared23.CUBE
        obstPared23.header.frame_id="map"
        obstPared23.id = 37
        obstPared23.ns ="Obstaculos"
        obstPared23.lifetime=rospy.Duration()
        obstPared23.action =   obstPared23.ADD
        obstPared23.scale.x = 2.7
        obstPared23.scale.y = 1.6
        obstPared23.scale.z = 0.5
        obstPared23.pose.position.x = -2.5
        obstPared23.pose.position.y = -5
        obstPared23.pose.position.z = 0.25
        obstPared23.pose.orientation.x = 0.0
        obstPared23.pose.orientation.y = 0.0
        obstPared23.pose.orientation.z = 0
        obstPared23.pose.orientation.w = 1
        obstPared23.color.a = 1
        obstPared23.color.r = 255/255
        obstPared23.color.g = 200/255
        obstPared23.color.b = 0/255

        obstPared24=Marker()
        obstPared24.type =obstPared24.CUBE
        obstPared24.header.frame_id="map"
        obstPared24.id = 38
        obstPared24.ns ="Obstaculos"
        obstPared24.lifetime=rospy.Duration()
        obstPared24.action =   obstPared24.ADD
        obstPared24.scale.x = 4
        obstPared24.scale.y = 0.8
        obstPared24.scale.z = 0.5
        obstPared24.pose.position.x = -2
        obstPared24.pose.position.y = -2.5
        obstPared24.pose.position.z = 0.25
        obstPared24.pose.orientation.x = 0.0
        obstPared24.pose.orientation.y = 0.0
        obstPared24.pose.orientation.z = 0
        obstPared24.pose.orientation.w = 1
        obstPared24.color.a = 1
        obstPared24.color.r = 255/255
        obstPared24.color.g = 200/255
        obstPared24.color.b = 0/255

        obstPared25=Marker()
        obstPared25.type =obstPared25.CUBE
        obstPared25.header.frame_id="map"
        obstPared25.id = 39
        obstPared25.ns ="Obstaculos"
        obstPared25.lifetime=rospy.Duration()
        obstPared25.action =   obstPared25.ADD
        obstPared25.scale.x = 1
        obstPared25.scale.y = 2.3
        obstPared25.scale.z = 0.5
        obstPared25.pose.position.x = 4.6
        obstPared25.pose.position.y = -1.6
        obstPared25.pose.position.z = 0.25
        obstPared25.pose.orientation.x = 0.0
        obstPared25.pose.orientation.y = 0.0
        obstPared25.pose.orientation.z = 0
        obstPared25.pose.orientation.w = 1
        obstPared25.color.a = 1
        obstPared25.color.r = 255/255
        obstPared25.color.g = 200/255
        obstPared25.color.b = 0/255

        obstPared26=Marker()
        obstPared26.type =obstPared26.CUBE
        obstPared26.header.frame_id="map"
        obstPared26.id = 40
        obstPared26.ns ="Obstaculos"
        obstPared26.lifetime=rospy.Duration()
        obstPared26.action =   obstPared26.ADD
        obstPared26.scale.x = 4
        obstPared26.scale.y = 0.8
        obstPared26.scale.z = 0.5
        obstPared26.pose.position.x = 1
        obstPared26.pose.position.y = -1.5
        obstPared26.pose.position.z = 0.25
        obstPared26.pose.orientation.x = 0.0
        obstPared26.pose.orientation.y = 0.0
        obstPared26.pose.orientation.z = 0
        obstPared26.pose.orientation.w = 1
        obstPared26.color.a = 1
        obstPared26.color.r = 255/255
        obstPared26.color.g = 200/255
        obstPared26.color.b = 0/255

        obstPared27=Marker()
        obstPared27.type =obstPared27.CUBE
        obstPared27.header.frame_id="map"
        obstPared27.id = 41
        obstPared27.ns ="Obstaculos"
        obstPared27.lifetime=rospy.Duration()
        obstPared27.action =   obstPared27.ADD
        obstPared27.scale.x = 6.3
        obstPared27.scale.y = 0.8
        obstPared27.scale.z = 0.5
        obstPared27.pose.position.x = 0.0
        obstPared27.pose.position.y = -0.5
        obstPared27.pose.position.z = 0.25
        obstPared27.pose.orientation.x = 0.0
        obstPared27.pose.orientation.y = 0.0
        obstPared27.pose.orientation.z = 0
        obstPared27.pose.orientation.w = 1
        obstPared27.color.a = 1
        obstPared27.color.r = 255/255
        obstPared27.color.g = 200/255
        obstPared27.color.b = 0/255


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

        if(prueba>=111 and prueba<=777):
            numObst=[]
            numObst.append(int(prueba)%10)
            numObst.append(int(prueba/10)%10)
            numObst.append(int(prueba/100)%10)
            for i in numObst:
                if(i==1):
                    pub_markers.publish(obstPared5)
                if(i==2):
                    pub_markers.publish(obstPared1)
                    pub_markers.publish(obstPared3)
                if(i==3):
                    pub_markers.publish(obstPared1)
                    pub_markers.publish(obstPared2)
                if(i==4):
                    pub_markers.publish(obstPared)
                    
                if(i==5):
                    pub_markers.publish(obstPared4)

                if(i==6):
                    pub_markers.publish(obstPared7)
                if(i==7):
                    pub_markers.publish(obstPared6)
        # la prueba 3 es del RRT, la prueba 31 y 32 es del PRM
        if(prueba==3 or prueba==31 or prueba==32):
            pub_markers.publish(obstPared)
            pub_markers.publish(obstPared5)
            pub_markers.publish(obstPared1)
            pub_markers.publish(obstPared2)
            pub_markers.publish(obstPared14)
            pub_markers.publish(obstPared8)
            pub_markers.publish(obstPared9)
            pub_markers.publish(obstPared10)
            pub_markers.publish(obstPared11)
            pub_markers.publish(obstPared12)
            pub_markers.publish(obstPared13)
            pub_markers.publish(obstPared15)
            pub_markers.publish(obstPared16)
            pub_markers.publish(obstPared17)
            pub_markers.publish(obstPared18)
            pub_markers.publish(obstPared19)
            pub_markers.publish(obstPared20)
            pub_markers.publish(obstPared21)
            pub_markers.publish(obstPared22)
            pub_markers.publish(obstPared23)
            
        """
        if(prueba==31):
            pub_markers.publish(obstPared5)
            pub_markers.publish(obstPared1)
            pub_markers.publish(obstPared2)
            pub_markers.publish(obstPared14)
            pub_markers.publish(obstPared8)
            pub_markers.publish(obstPared9)
            pub_markers.publish(obstPared10)
            pub_markers.publish(obstPared11)
            pub_markers.publish(obstPared12)
            pub_markers.publish(obstPared13)
            pub_markers.publish(obstPared17)
            pub_markers.publish(obstPared19)
            pub_markers.publish(obstPared20)
            #pub_markers.publish(obstPared24)
            #pub_markers.publish(obstPared25)


        if(prueba==32):
            pub_markers.publish(obstPared)
            pub_markers.publish(obstPared8)
            pub_markers.publish(obstPared9)            
            pub_markers.publish(obstPared12)
            pub_markers.publish(obstPared13)
            pub_markers.publish(obstPared15)
            pub_markers.publish(obstPared16)
            pub_markers.publish(obstPared18)
            pub_markers.publish(obstPared21)
            pub_markers.publish(obstPared22)
            pub_markers.publish(obstPared23)
            #pub_markers.publish(obstPared26)
            #pub_markers.publish(obstPared27)
        """




        for i in range(3):
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
            
            if(prueba>=111 and prueba<=777):
                numObst=[]
                numObst.append(int(prueba)%10)
                numObst.append(int(prueba/10)%10)
                numObst.append(int(prueba/100)%10)
                for i in numObst:
                    if(i==1):
                        pObst.append(obstPared5)
                    if(i==2):
                        
                        pObst.append(obstPared1)
                        pObst.append(obstPared3)
                    if(i==3):
                        pObst.append(obstPared1)
                        pObst.append(obstPared2)
                    if(i==4):
                        pObst.append(obstPared)
                        
                    if(i==5):
                        pObst.append(obstPared4)

                    if(i==6):
                        pObst.append(obstPared7)
                    if(i==7):
                        pObst.append(obstPared6)

            if(prueba==3):
                pObst.append(obstPared)
                pObst.append(obstPared5)
                pObst.append(obstPared1)
                pObst.append(obstPared2)
                pObst.append(obstPared14)
                pObst.append(obstPared8)
                pObst.append(obstPared9)
                pObst.append(obstPared10)
                pObst.append(obstPared11)
                pObst.append(obstPared13)
                pObst.append(obstPared15)
                pObst.append(obstPared16)
                pObst.append(obstPared17)
                pObst.append(obstPared18)
                pObst.append(obstPared19)
                pObst.append(obstPared20)
                pObst.append(obstPared21)
                pObst.append(obstPared22)
                pObst.append(obstPared23)


            if(prueba==31):
                pObst.append(obstPared5)
                pObst.append(obstPared1)
                pObst.append(obstPared2)
                pObst.append(obstPared14)
                pObst.append(obstPared8)
                pObst.append(obstPared9)
                pObst.append(obstPared10)
                pObst.append(obstPared11)
                pObst.append(obstPared12)
                pObst.append(obstPared13)
                pObst.append(obstPared17)
                pObst.append(obstPared19)
                pObst.append(obstPared20)
                pObst.append(obstPared24)
                pObst.append(obstPared25)
               


            if(prueba==32):
                pObst.append(obstPared)
                pObst.append(obstPared8)
                pObst.append(obstPared9)            
                pObst.append(obstPared12)
                pObst.append(obstPared13)
                pObst.append(obstPared15)
                pObst.append(obstPared16)
                pObst.append(obstPared18)
                pObst.append(obstPared21)
                pObst.append(obstPared22)
                pObst.append(obstPared23)
                pObst.append(obstPared26)
                pObst.append(obstPared27)   
                


        return pObst



    def Rutas(p1,p2,pub_makers,cont_id,tipoRob):
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

        unirPuntos.color.a=0.5
        if(tipoRob==1):
            unirPuntos.color.r=130/255
            unirPuntos.color.g=0/255
            unirPuntos.color.b=0/255
        elif(tipoRob==2):
            unirPuntos.color.r=0/255
            unirPuntos.color.g=0/255
            unirPuntos.color.b=0/255
        
        

        unirPuntos.points.append(p1)
        unirPuntos.points.append(p2)
        

        pub_makers.publish(unirPuntos)

    def RutaLibre(p1,p2,pub_makers,cont_id,tipoRob):
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

        unirPuntos.color.a=1
        if(tipoRob==1):
            unirPuntos.color.r=255/255
            unirPuntos.color.g=0/255
            unirPuntos.color.b=0/255
        elif(tipoRob==2):
            unirPuntos.color.r=0/255
            unirPuntos.color.g=0/255
            unirPuntos.color.b=0/255

        unirPuntos.points.append(p1)
        unirPuntos.points.append(p2)
        

        pub_makers.publish(unirPuntos)
