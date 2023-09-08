#!/usr/bin/env python3

# Librerias necesarias
import rospy
import numpy as np
from std_msgs.msg import Float64
from roslib import packages
from functions import *
import rbdl 

# Se definen variables iniciales
bandera_pd= False
tmax = 180
epsilon =1e-3


if __name__ == "__main__":

    # Se inicializa el nodo
    rospy.init_node("sendJointsGzNode")
    topic1 = '/robot/joint1_position_controller/command'
    topic2 = '/robot/joint2_position_controller/command'
    topic3 = '/robot/joint3_position_controller/command'
    topic4 = '/robot/joint4_position_controller/command'
    topic5 = '/robot/joint5_position_controller/command'
    topic6 = '/robot/joint6_position_controller/command'
    topic7 = '/robot/joint7_position_controller/command'
    pub1 = rospy.Publisher(topic1, Float64, queue_size=10, latch=True)
    pub2 = rospy.Publisher(topic2, Float64, queue_size=10, latch=True)
    pub3 = rospy.Publisher(topic3, Float64, queue_size=10, latch=True)
    pub4 = rospy.Publisher(topic4, Float64, queue_size=10, latch=True)
    pub5 = rospy.Publisher(topic5, Float64, queue_size=10, latch=True)
    pub6 = rospy.Publisher(topic6, Float64, queue_size=10, latch=True)
    pub7 = rospy.Publisher(topic7, Float64, queue_size=10, latch=True)

    # Se definen variables
    q1 = Float64()
    q2 = Float64()
    q3 = Float64()
    q4 = Float64()
    q5 = Float64()
    q6 = Float64()
    q7 = Float64()

    # Variables para guardar data
    fqact = open("/tmp/qactual.dat", "w")
    fqdes = open("/tmp/qdeseado.dat", "w")
    fxact = open("/tmp/xactual.dat", "w")
    fxdes = open("/tmp/xdeseado.dat", "w")        
    fu = open("/tmp/esfcontrol.dat", "w")     

    # Variables iniciales del dispositivo
    q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])    
    dq = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    qdes = np.array([0.0, -45.0, 0.0, 0.0, 45.0, 45.0, 180.0])
    xdes = cdir(qdes)[0:3, 3]

    # Se carga el modelo urdf
    modelo = rbdl.loadModel('../urdf/prueba_gazebo.urdf')
    ndof = modelo.q_size

    freq= 500
    dt = 1.0/freq
    rate = rospy.Rate(freq)

    # Simulador dinámico del dispositivo
    robot= Robot(q, dq, ndof, dt)

    # Ganancias de los controladores
    valores = 50*np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])      
    Kp = np.diag(valores)
    Kd = 20*np.sqrt(Kp) # 25

    u = np.zeros(ndof)         # Esfuerzo de control 
    M = np.zeros([ndof, ndof]) # Matriz de inercia 
    g = np.zeros(ndof)         # Gravedad
    zeros = np.zeros(ndof)     
    c = np.zeros(ndof)         # Coriolis

    t= 0.0

    while not rospy.is_shutdown():

        if(bandera_pd == False) and (round(t,4) <= round(tmax, 4)):

            # Se obtiene las posiciones y velocidades angulares
            q = robot.read_joint_positions() #sexagesimales
            dq = robot.read_joint_velocities()
            
            x = cdir(q)[0:3,3]
            
            # Se guarda data 
            fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
            fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
            fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+ ' '+str(q[6])+ '\n ')
            fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+' '+str(qdes[6])+'\n ')
            fu.write(str(t)+' '+str(u[0])+' '+str(u[1])+' '+ str(u[2])+' '+ str(u[3])+' '+str(u[4])+' '+str(u[5])+' '+str(u[6])+'\n ')

            # Error
            e= qdes- q 
            print("Error: {}".format(np.round(np.linalg.norm(e),4)))

            if (np.linalg.norm(e)<epsilon) or (round(t,4) == round(tmax,4)):
                bandera_pd = True
                print("Posicion deseada alcanzada")

            rbdl.InverseDynamics(modelo, q, zeros, zeros, g)

            u = g + Kp.dot(qdes-q) - Kp.dot(dq)   # Ley de control

            robot.send_command(u)       # Se entrega el esfuerzo de control

            q = q.dot(pi/180)           # Conversión de sexagesimal a radianes

            q1= q[0]; q2=q[1]; q3=q[2]; q4=q[3]; q5=q[4]; q6=q[5]; q7=q[6]

            # Se actualizan los vaalores en Gazebo
            pub1.publish(q1)
            pub2.publish(q2)
            pub3.publish(q3)
            pub4.publish(q4)
            pub5.publish(q5)
            pub6.publish(q6)
            pub7.publish(q7)

            q = q.dot(180/pi)           # Conversión de radianes a sexagesimales
            t = t+dt 

            rate.sleep()
        
        else: 

            pub1.publish(q1)
            pub2.publish(q2)
            pub3.publish(q3)
            pub4.publish(q4)
            pub5.publish(q5)
            pub6.publish(q6)
            pub7.publish(q7)
            t= t+dt 

            rate.sleep()

