#!/bin/bash


p1() { 
    echo " "
    echo "*****************************************************************************************************"
    echo "                                               PRUEBA 1                                              "
    echo "Esta prueba permite variar las coordenadas de Salida y Llegada de los robots con osbtaculos fijos"
    echo "*****************************************************************************************************"
    echo " "

    PS3="Seleccione un algoritmo: "
    select opt in Algoritmo_RRT Algoritmo_PRM Atras; 


do 

	case $opt in 

		Algoritmo_RRT) 
		    echo " "
			echo "*****************************************************************************************************"
			echo "                                    EJECUTANDO ALGORITMO RRT                                         "
			echo "*****************************************************************************************************"
			echo " "
			sigma=0.7
			N=1000
			robot=1;prueba=1
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_RRT.py $N $robot $prueba $sigma; bash" &
			sleep 1
			robot=2
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_RRT.py $N $robot $prueba $sigma; bash" &
			sleep 10
			menu
		 ;; 

		Algoritmo_PRM) 
		
		    echo " "
			echo "*****************************************************************************************************"
			echo "                                    EJECUTANDO ALGORITMO PRM                                         "
			echo "*****************************************************************************************************"
			echo " "
			N=1000
			prueba=1
			robot=1;
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_PRM.py $N $robot $prueba; bash" &
			sleep 1
			robot=2;
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_PRM.py $N $robot $prueba; bash" &
			sleep 10
			menu
		
		;; 

		Atras) 

			menu;; 

		*) echo "$REPLY es una opción inválida";; 

	esac 

done


	#read -p "Ingrese la coordenada del robot 1 -> x y: " x1 y1
	#read -p "Ingrese la coordenada del robot 2 -> x y: " x2 y2

	#echo "$y1 + $y2 = " $(bc -l <<< "$y1+$y2") 
} 


p2() { 
    echo " "
    echo "*****************************************************************************************************"
    echo "                                               PRUEBA 2                                              "
    echo "Esta prueba varia las coordenadas de Salida y Llegada de los robots con obstaculos aleatorios"
    echo "*****************************************************************************************************"
    echo " "
	#Obstaculos random
	echo "Los obstaculos arbitrarios son los restangulos que estan delineados"
	
	n1=$(($RANDOM % 7 + 1))
	n2=$(($RANDOM % 7 + 1))
	n3=$(($RANDOM % 7 + 1))
	randObst=$(bc -l <<< "$n1*100+$n2*10+$n3") 
	echo " "
	echo "Los obstaculos que arbitrarios son: el $n1, el $n2 y el $n3"
	ascii-image-converter LOGO_ENTORNO.png -d 40,20 -b

    PS3="Seleccione un algoritmo: "
    select opt in Algoritmo_RRT Algoritmo_PRM Atras; 
do 

	case $opt in 

		Algoritmo_RRT) 
		    echo " "
			echo "*****************************************************************************************************"
			echo "                                    EJECUTANDO ALGORITMO RRT                                         "
			echo "*****************************************************************************************************"
			echo " "
			sigma=0.9
			N=1000
			robot=1;prueba=$randObst
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_RRT.py $N $robot $prueba $sigma; bash" &
			sleep 1
			robot=2
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_RRT.py $N $robot $prueba $sigma; bash" &
			sleep 10
			menu
		 ;; 

		Algoritmo_PRM) 
		
		    echo " "
			echo "*****************************************************************************************************"
			echo "                                    EJECUTANDO ALGORITMO PRM                                         "
			echo "*****************************************************************************************************"
			echo " "
			N=1000
			prueba=$randObst
			robot=1;
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_PRM.py $N $robot $prueba; bash" &
			sleep 1
			robot=2;
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_PRM.py $N $robot $prueba; bash" &
			sleep 10
			menu
		
		;; 

		Atras) 

			menu;; 

		*) echo "$REPLY es una opción inválida";; 

	esac 

done
} 


#**************************************PRUEBA 3
p3() { 
    echo " "
    echo "*****************************************************************************************************"
    echo "                                               PRUEBA 3                                              "
    echo "              En esta prueba el usuario puede variar los parametros de los algoritmos"
	echo "                                      en el entorno laberinto"
    echo "*****************************************************************************************************"
    echo " "
	

    PS3="Seleccione un algoritmo: "
    select opt in Algoritmo_RRT Algoritmo_PRM Atras; 


do 

	case $opt in 

		Algoritmo_RRT) 
		    echo " "
			echo "*****************************************************************************************************"
			echo "                                    EJECUTANDO ALGORITMO RRT                                         "
			echo "*****************************************************************************************************"
			echo " "
			read -p "Ingrese el numero de itraciones: " nIt
			read -p "Ingrese el factor sigma [0.1 0.9] para el Robot 1: " nSig
			N=$nIt
			sigma1=$nSig
			read -p "Ingrese el factor sigma [0.1 0.9] para el Robot 2: " nSig
			sigma2=$nSig
			prueba=3
			robot=1;
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_RRT.py $N $robot $prueba $sigma1; bash" &
			sleep 1
			robot=2;
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_RRT.py $N $robot $prueba $sigma2; bash" &
			sleep 10
			menu
		 ;; 

		Algoritmo_PRM) 
		    echo " "
			echo "*****************************************************************************************************"
			echo "                                    EJECUTANDO ALGORITMO PRM                                         "
			echo "*****************************************************************************************************"
			echo " "
			read -p "Ingrese el numero de itraciones del Robot 1: " nIt
			N1=$nIt
			read -p "Ingrese el numero de itraciones del Robot 2: " nIt
			N2=$nIt
			prueba=3
			robot=1;
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_PRM.py $N1 $robot $prueba; bash" &
			sleep 1
			robot=2;
			gnome-terminal -- sh -c "cd;cd catkin_ws/src/path_planning_rrt_prm/scripts;python3 Pruebas_PRM.py $N2 $robot $prueba; bash" &
			sleep 10
			menu
		
		;; 

		Atras) 

			menu;; 

		*) echo "$REPLY es una opción inválida";; 

	esac 

done
} 


menu(){
    
clear
ascii-image-converter LOGO_EPN.png -d 100,12 -b

echo "                       APLICACIÓN DE ALGORITMOS ITERATIVOS DE PLANIFICACIÓN"
echo "          DE RUTAS (PATH PLANNING) EN UN SISTEMA MULTI-AGENTE DENTRO DEL AMBIENTE DE ROS-GAZEBO"
echo " "
echo "          AUTOR: EDISON ANDRES SANTILLAN PULLUTASIG"
echo " "
PS3="Seleccione una prueba: " 


select opt in Prueba_1_Obstaculos_Fijos Prueba_2_Obstaculos_Aleatorios  Prueba_3_Laberinto Salir; 



do 

	case $opt in 

		Prueba_1_Obstaculos_Fijos) p1 ;; 

		Prueba_2_Obstaculos_Aleatorios) p2;; 

		Prueba_3_Laberinto) p3;; 

		Salir) 
			break;; 

		*) echo "$REPLY es una opción inválida"

			;; 

	esac 

done
}


menu