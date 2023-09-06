#!/usr/bin/env python3

# Librerias necesarias
import rospy
from sensor_msgs.msg import JointState
from markers import *
from functions import *
from roslib import packages
import numpy as np
import rbdl 

# Se definen variables 
Flag_pd = False
tmax = 180
epsilon = 1e-3

## Se inicializa el nodo 
rospy.init_node("control_PDgr")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])

# Se crean variables para guardar las posiciones deseadas y actuales
fqact = open("/tmp/qactual.dat", "w")
fqdes = open("/tmp/qdeseado.dat", "w")
fxact = open("/tmp/xactual.dat", "w")
fxdes = open("/tmp/xdeseado.dat", "w")

# Se nombra cada articulación
jnames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

# Configuración para enviar las posiciones a las articulaciones
jstate = JointState()

jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# Configuración articular inicial
q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  #q = np.array([0.0, 45.0, 0.0, 45.0, 45.0, 0.0, 0.0])

# Velocidad inicial 
dq = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Configuración articular deseada 
qdes = np.array([45.0, 0.0, 0.0, 45.0, 45.0, 0.0, 0.0])   # np.array([0.0, -45.0, 30.0, 0.0, -45.0, 45.0, 0.0])

# Posición de la conf. deseada
xdes = cdir(qdes)[0:3, 3]

# Ángulos en radianes
q = q.dot(pi/180)

# Se publica el mensaje con la posición articular
jstate.position = q
pub.publish(jstate)

q = q.dot(180/pi)

# Se obtine el modelo por RBDL
modelo = rbdl.loadModel('../urdf/otro.urdf')
ndof = modelo.q_size 

# Frecuencia para el envío
freq = 500
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinámico del robot
robot= Robot(q, dq, ndof, dt)

# Ganancias de los controladores
valores = 40*np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])  # 50 10  // 5 2
Kp = np.diag(valores)
Kd = 25*np.sqrt(Kp) #2

u = np.zeros(ndof)         # Esfuerzo de control 
M = np.zeros([ndof, ndof]) # Matriz de inercia 
g = np.zeros(ndof)         # Gravedad
zeros = np.zeros(ndof)     
c = np.zeros(ndof)         # Coriolis

t = 0.0
while not rospy.is_shutdown():

	if (Flag_pd == False) and (round(t,4) <= round(tmax,4)):

		# Se obtienen las posiciones y velocidades del dispositivo 
		q = robot.read_joint_positions()		
		dq = robot.read_joint_velocities()
		x = cdir(q)[0:3,3]
		
		jstate.header.stamp = rospy.Time.now()

		# Se guarda la data
		fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
		fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
		fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+ ' '+str(q[6])+ '\n ')
		fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+' '+str(qdes[6])+'\n ')
		
		# Error 
		e = qdes - q
		print("Error: {}".format(np.round(np.linalg.norm(e),4)));

		if (np.linalg.norm(e)<epsilon) or (round(t,4) == round(tmax,4)):
			Flag_pd = True
			print("Posicion deseada alcanzada")

		rbdl.InverseDynamics(modelo, q, zeros, zeros, g)  # Vector de gravedad

		# Ley de control
		u = g + Kp.dot(qdes-q) - Kp.dot(dq)
		
		# Retroalimentación para el simulador	
		robot.send_command(u)

		q = q.dot(pi/180)
		jstate.position = q
		q = q.dot(180/pi)

		pub.publish(jstate)
		bmarker_deseado.xyz(xdes)
		bmarker_actual.xyz(x)
		t = t+dt

		# Esperar hasta la siguiente iteracion
		rate.sleep()

	#Mantener los ultimos valores de articulacion si ya se alcanzaron los valores deseados
	else:

		# Publicacion del mensaje
		jstate.position = q
		pub.publish(jstate)
		bmarker_deseado.xyz(xdes)
		bmarker_actual.xyz(x)
		t = t+dt
		# Esperar hasta la siguiente iteracion
		rate.sleep()











