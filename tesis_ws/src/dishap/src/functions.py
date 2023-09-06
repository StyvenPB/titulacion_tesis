# Librerías necesarias 
import numpy as np 
from copy import copy
import rbdl
from roboclaw_3 import Roboclaw

# Se definen las funciones trigonométicas 
cos= np.cos; sin=np.sin; pi=np.pi 

# Medidas de cada eslabón en metros (m)
l1= 67.75/100; l2=10/100; l3=7.75/100; l4=15/100; l5=12.25/100; l6=7.75/100; l7=10/100; l8=12.25/100; l9=12.25/100  

# Funciones creadas
def dh(d, theta, a, alpha):		# Parámetros Denavit-Hartenberg para matriz de transformación.
	
	theta=np.deg2rad(theta)
	alpha=np.deg2rad(alpha)

	T= np.array([[cos(theta), -cos(alpha)*sin(theta),  sin(alpha)*sin(theta), a*cos(theta)], 
		[sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
		[0, sin(alpha), cos(alpha), d],
		[0, 0, 0, 1]])
	
	return T


def cdir(q):  # Cinemática directa. 
	
	T01= dh(l1, -90+q[0], -l2,  90)
	T12= dh(0,  180+q[1],  l3,  90)
	T23= dh(l4,  90+q[2],   0,  90)
	T34= dh(l5, 270+q[3],  l6,  0)
	T45= dh(l7,  90+q[4],   0,  90)
	T56= dh(l8, 180+q[5],   0,  90)
	T67= dh(l9,   0+q[6],   0,  0)

	T02= T01.dot(T12)
	T03= T02.dot(T23)
	T04= T03.dot(T34)
	T05= T04.dot(T45)
	T06= T05.dot(T56)
	T07= T06.dot(T67)

	return T07


def jacobiano_haptic(q, delta=0.0001):

	# Jacobiano analítico para la posición
	J = np.zeros((3,7))
	q = q.dot(180/pi)
	T = cdir(q)
	q = q.dot(pi/180)

	for i in range(7):

		dq = copy(q)
		dq[i]=dq[i]+delta

		dq = dq.dot(180/pi)
		Td = cdir(dq)
		dq = dq.dot(pi/180)

		J[0,i]= (1/delta)*(Td[0,3]-T[0,3])
		J[1,i]= (1/delta)*(Td[1,3]-T[1,3])
		J[2,i]= (1/delta)*(Td[2,3]-T[2,3])

	return J


def cinv(xdes, q0):		# Cinemática inversa

	epsilon= 0.001
	max_iter=1000
	delta= 0.0001

	q= copy(q0)

	for i in range(max_iter):
		J=jacobiano_haptic(q, delta=0.0001)
		q = q.dot(180/pi)
		f= cdir(q)
		q = q.dot(pi/180)
		e= xdes-f[0:3,3]
		q = q+np.dot(np.linalg.pinv(J),e)
		
		if(np.linalg.norm(e)<epsilon):
			break
	return q


def leer_encoder(rc,address):		# Leer velocidad y posición de motores
	# M1 
	ce1= 9600 
	q1 = rc[0].ReadEncM1(address)
	q1 = q1[1]*(360/ce1)

	vmax1 = 11100 
	dq1 = rc[0].ReadSpeedM1(address)
	dq1 = dq1[1]*(100/vmax1)*(360/60)

	# M2 
	ce2= 9600 
	q2 = rc[0].ReadEncM2(address)
	q2 = q2[1]*(360/ce2)

	vmax2 = 11100 
	dq2 = rc[0].ReadSpeedM2(address)
	dq2 = dq2[1]*(100/vmax2)*(360/60)
	
	# M3
	ce3 = 9600
	q3 = rc[1].ReadEncM1(address)
	q3 = q3[1]*(360/ce3)

	vmax3 = 11100 
	dq3 = rc[1].ReadSpeedM1(address)
	dq3 = dq2[1]*(100/vmax3)*(360/60)

	# M4
	ce4 = 6533
	q4 = rc[1].ReadEncM2(address)
	q4 = q4[1]*(360/ce4)

	vmax4 = 16560 
	dq4 = rc[1].ReadSpeedM2(address)
	dq4 = dq4[1]*(100/vmax4)*(360/60)

	# M5
	ce5 = 980
	q5 = rc[2].ReadEncM1(address)
	q5 = q5[1]*(360/ce5)

	vmax5 = 82830 
	dq5 = rc[2].ReadSpeedM1(address)
	dq5 = dq5[1]*(100/vmax5)*(360/60)

	# M6
	ce6 = 980
	q6 = rc[2].ReadEncM2(address)
	q6 = q6[1]*(360/ce6)

	vmax6 = 82830 
	dq6 = rc[2].ReadSpeedM2(address)
	dq6 = dq6[1]*(100/vmax6)*(360/60)

	q = [q1, q2, q3, q4, q5, q6]
	dq = [dq1, dq2, dq3, dq4, dq5, dq6]
	return [q, dq]


def input_control(vel, ang, rc):
	
	# M1
	vmax1 = 11100       # Velocidad máxima leida por encoder
	vmax_rpm1 = 67     # Velocidad máxima del motor en RPM
	vel1 = vel[0]
	vel_rpm1 = np.abs(vel1)*60/360   # RPM
	vel_m1 = np.round(vel_rpm1/(vmax_rpm1/vmax1),0)
	vel_m1 = int(vel_m1)

	ce1 = 9600         # Numero de cuentas del encoder
	ang1 = ang[0] 
	ang_m1 = np.round(ang1/(360/ce1), 0)
	ang_m1 = int(ang_m1)

	rc[0].SpeedAccelDeccelPositionM1(0x80,10000,vel_m1,10000, ang_m1, 1) # Enviar velocidad y posicion 

	# M2
	vmax2 = 11100       # Velocidad máxima leida por encoder
	vmax_rpm2 = 67      # Velocidad máxima del motor en RPM
	vel2 = vel[1]
	vel_rpm2 = np.abs(vel2)*60/360   # RPM
	vel_m2 = np.round(vel_rpm2/(vmax_rpm2/vmax2),0)
	vel_m2 = int(vel_m2)

	ce2 = 9600          # Numero de cuentas del encoder
	ang2 = ang[1]
	ang_m2 = np.round(ang2/(360/ce2), 0)
	ang_m2 = int(ang_m2)
	
	rc[0].SpeedAccelDeccelPositionM2(0x80,10000,vel_m2,10000, ang_m2, 1) # Enviar velocidad y posicion

	# M3 
	vmax3 = 11100        # Velocidad máxima leida por encoder
	vmax_rpm3 = 67      # Velocidad máxima del motor en RPM
	vel3 = vel[2]
	vel_rpm3 = np.abs(vel3)*60/360   # RPM
	vel_m3 = np.round(vel_rpm3/(vmax_rpm3/vmax3),0)
	vel_m3 = int(vel_m3)

	ce3 = 9600          # Numero de cuentas del encoder
	ang3 = ang[2]
	ang_m3 = np.round(ang3/(360/ce3), 0)
	ang_m3 = int(ang_m3)
	
	rc[1].SpeedAccelDeccelPositionM1(0x80,10000,vel_m3,10000, ang_m3, 1) # Enviar velocidad y posicion 

	# M4
	vmax4 = 16500         # Velocidad máxima leida por encoder
	vmax_rpm4 = 100      # Velocidad máxima del motor en RPM
	vel4 = vel[3]
	vel_rpm4 = np.abs(vel4)*60/360   # RPM
	vel_m4 = np.round(vel_rpm4/(vmax_rpm4/vmax4),0)
	vel_m4 = int(vel_m4)

	ce4 = 6533            # Numero de cuentas del encoder
	ang4 = ang[3]
	ang_m4 = np.round(ang4/(360/ce4), 0)
	ang_m4 = int(ang_m4)
	
	rc[1].SpeedAccelDeccelPositionM2(0x80,10000,vel_m4,10000, ang_m4, 1)   # Enviar velocidad y posicion 

	# M5
	vmax5 = 82830        # Velocidad máxima leida por encoder
	vmax_rpm5= 500      # Velocidad máxima del motor en RPM
	vel5 = vel[4]
	vel_rpm5 = np.abs(vel5)*60/360   # RPM
	vel_m5 = np.round(vel_rpm5/(vmax_rpm5/vmax5),0)
	vel_m5 = int(vel_m5)

	ce5 = 980  # Numero de cuentas del encoder
	ang5 = ang[4]
	ang_m5 = np.round(ang5/(360/ce5), 0)
	ang_m5 = int(ang_m5)
	
	rc[2].SpeedAccelDeccelPositionM1(0x80,10000,vel_m5,10000, ang_m5, 1)  # Enviar velocidad y posicion 

	# M6
	vmax6 = 82830        # Velocidad máxima leida por encoder
	vmax_rpm6 = 500     # Velocidad máxima del motor en RPM
	vel6 = vel[5]
	vel_rpm6 = np.abs(vel6)*60/360   # RPM
	vel_m6 = np.round(vel_rpm6/(vmax_rpm6/vmax6),0)
	vel_m6 = int(vel_m6)

	ce6 = 980  # Numero de cuentas del encoder
	ang6 = ang[5]
	ang_m6 = np.round(ang6/(360/ce6), 0)
	ang_m6 = int(ang_m6)
	
	rc[2].SpeedAccelDeccelPositionM2(0x80,10000,vel_m6, 10000, ang_m6, 1)   # Enviar velocidad y posicion 


class Robot(object):		# Función para simulador de pruebas 
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/prueba_gazebo.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq


def dh_ur5(d, theta, a, alfa): 		# Matriz de transformación homogénea UR5
	
	T = np.array([[cos(theta), -cos(alfa)*sin(theta), sin(alfa)*sin(theta), a*cos(theta)],
	[sin(theta), cos(alfa)*cos(theta), -sin(alfa)*cos(theta), a*sin(theta)],
	[0, sin(alfa), cos(alfa), d],
	[0, 0, 0, 1]])

	return T


def fkine_ur5(q):			# Cinemática directa UR5

	T01 = dh_ur5(0.0892,  +q[0],   0.0,    +pi/2)
	T12 = dh_ur5(0.0,     +q[1],   -0.425, 0.0)
	T23 = dh_ur5(0.0,     +q[2],   -0.392, 0.0)
	T34 = dh_ur5(0.1093,  pi+q[3], 0.0,    -pi/2)
	T45 = dh_ur5(0.09475, +q[4],   0.0,    +pi/2)
	T56 = dh_ur5(0.0825,  +q[5],   0.0,    0.0)

	T02 = T01.dot(T12)
	T03 = T02.dot(T23)
	T04 = T03.dot(T34)
	T05 = T04.dot(T45)
	T06 = T05.dot(T56)

	T = T06
	return T


def jacobian_ur5(q, delta=0.0001):
	J = np.zeros((3,6))
	T= fkine_ur5(q)
	for i in range(6):
		dq = copy(q)
		dq[i]= dq[i]+delta
		Td=fkine_ur5(dq)
		# Aproximacion del Jacobiano
		J[0,i]= (1/delta)*(Td[0,3]-T[0,3])
		J[1,i]= (1/delta)*(Td[1,3]-T[1,3])
		J[2,i]= (1/delta)*(Td[2,3]-T[2,3])
	return J


def ikine_ur5(xdes, q0):			# Cinemática inversa UR5

	epsilon = 0.001
	max_iter = 1000
	delta = 0.00001
	q = copy(q0)
	for i in range(max_iter):
		J=jacobian_ur5(q, delta=0.0001)
		f=fkine_ur5(q)
		e= xdes-f[0:3,3]
		q= q+ np.dot(np.linalg.pinv(J),e)

		if(np.linalg.norm(e)<epsilon):
			break
	
	return q


def trayectoria(p_dmact, p_dm, p_ur5, q_ur5):			# Generación de trayectoria
    
    cambio = [p_dmact[0,3]-p_dm[0,3], p_dmact[1,3]-p_dm[1,3], p_dmact[2,3]-p_dm[2,3]] 
    cambio = np.dot(cambio,(1/3)) # Escalamiento 1:3
    
    xdes = [p_ur5[0,3]+cambio[0], p_ur5[1,3]+cambio[1], p_ur5[2,3]+cambio[2]]
    q_ur5req = ikine_ur5(xdes, q_ur5)

    return q_ur5req

