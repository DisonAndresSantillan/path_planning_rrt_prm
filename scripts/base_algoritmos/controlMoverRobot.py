#!/usr/bin/env python3
import rospy, math
from geometry_msgs.msg import Twist, Pose2D,Point #To publish velocities and tracking errors using the Pose2D message type
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, pi

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

#Initialize variables (robot posture) and the offset of the outside point
x = 0
y = 0
yaw = 0
l = 0.1

vel_msg = Twist()
error_msg = Pose2D()
t = 0.0; t0 = 0.0; V_max = 0.22; W_max = 2.84; #Timer, initial time, maximum velocities [m/s, rad/s], respectively, for a burger type
T = 100.0; k = 0.65; #Trajectory period, controller gains kx = ky = k
ex = 0.0; ey = 0.0; V = 0.0; W = 0.0

def getKey(): #Function to use keyboard events on Linux
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def odomCallback(msg): #Callback function to get the robot posture
	global x; global y; global yaw
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	#Operations to convert from quaternion to Euler angles (and vice-versa)
	quater = msg.pose.pose.orientation
	quater_list = [quater.x, quater.y, quater.z, quater.w]
	(roll, pitch, yaw) = euler_from_quaternion(quater_list) #Euler angles are given in radians
	#quat = quaternion_from_euler(roll, pitch,yaw); print quat
	
def velocity_controller(dict,Id): #Function to generate the desired trajectory and to compute the signals control
	global ex, ey, V, W, Xd,Yd #Indicate that some variables are global to be used in the main_function
	#Desired trajectory: Lemniscate
	a = 1; b = 0.5; X0 = 0; Y0 = 0; w = 2*pi/T
	#Desired position on the plane



	Xd = x+(dict[Id][0]-x)*t
	Yd = y+(dict[Id][1]-y)*t
	
	#Corresponding time derivatives
	Xdp = (dict[Id][0]-x)
	Ydp = (dict[Id][1]-y)
	
	p_x = x+l*cos(yaw); p_y = y+l*sin(yaw) #Compute the coordinates of the outside point
	ex = p_x-Xd; ey = p_y-Yd; #Compute tracking errors
	
	#Cinematic controller. Auxiliar controls, in global coordinates
	Ux = Xdp-k*ex; Uy = Ydp-k*ey
	
	#Compute the velocities according to the cinematic model for a differential type mobile robot
	V = Ux*cos(yaw)+Uy*sin(yaw)
	W = -Ux*sin(yaw)/l+Uy*cos(yaw)/l
	
	#Velocities saturation
	if (abs(V)>V_max):
		V = V_max*abs(V)/V
		#print("Sat V\t")
	if (abs(W)>W_max):
		W = W_max*abs(W)/W
		#print("Sat W\t")


def main_function():
	rospy.init_node('diff_robot_controller', anonymous=False) #Initialize the node
	rate = rospy.Rate(100) #Node frequency (Hz)
	
	vel_pub = rospy.Publisher('/r1/cmd_vel', Twist, queue_size=10) #To publish in the topic
	rospy.Subscriber('/r1/odom',Odometry, odomCallback) #To subscribe to the topic

	error_pub = rospy.Publisher('/tracking_errors', Pose2D, queue_size=10)
	vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0
	
	error_msg.theta = 0 #Since the orientation angle is a sub-actuated state, this field is assigned equal to zero

	
	print("To finish this node and to stop the robot, please press 'ctrl+C' or 'q' key\n")
	rospy.logwarn("To start the movement, the simulation must be running\n\n")
	
	global t, t0 #t and t0 are global to be used in velocity_controller
	t0 = rospy.Time.now().to_sec()
	contId=1
	
	while(1):
		t = rospy.Time.now().to_sec()-t0 #Compute the controller time  
		dict={}
		dict.setdefault(0,(1,1))
		dict.setdefault(1,(3,3))
		dict.setdefault(2,(-5,-5))
		dict.setdefault(3,(8,8))
		contId+=1
		if(contId==len(dict)):
			contId=1
		print("contid===========================: ",contId)

		velocity_controller(dict,contId) #Compute the control signals
		
		vel_msg.linear.x = V; vel_msg.angular.z = W
		vel_pub.publish(vel_msg); #Publish the control signals
		p1=Point();p2=Point()
		p1.x=dict[contId][0];p1.y=dict[contId][1]
		p2.x=x;p2.y=y


		while(distanciaEuclideana(p1,p2)>=0.1):
			p1.x=dict[contId][0];p1.y=dict[contId][1]
			p2.x=x;p2.y=y
			velocity_controller(dict,contId)
			vel_msg.linear.x = V; vel_msg.angular.z = W
			vel_pub.publish(vel_msg)
			print("xRob: ",x,"\t yRob: ",y,"\txRef: ",dict[contId][0],"\tyRef: ",dict[contId][1])

		error_msg.x = ex; error_msg.y = ey; #Assign the tracking errors
		error_pub.publish(error_msg); #Publish the tracking errors
		
		key = getKey()
		if(key == 'q' or key == '\x03'): #\x03 is ctrl+C
			break
		
		rate.sleep() #spinOnce() function does not exist in python
	
	#Stop the mobile robot
	vel_msg.linear.x = 0.0; vel_msg.angular.z = 0.0
	vel_pub.publish(vel_msg)
	
	print("Robot stops, but simulation keeps running\n")

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

if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        main_function() #Execute the function
    except rospy.ROSInterruptException:
        pass

    #if os.name != 'nt':
    #   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
