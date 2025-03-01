import math
import numpy as np
import matplotlib.pyplot as plt
from InvertedPendulum import InvertedPendulum

pend = InvertedPendulum()
pend.calc_discrete_system()
# parameter
r_wheel = pend.r_wheel
kb = pend.kb
gear_ratio = pend.gear_ratio
Rm = pend.Rm

Q = np.array([[10, 0, 0, 0], 
              [0, 10, 0 ,0], 
              [0, 0, 1, 0], 
              [0, 0, 0, 1]])
R = 100.0
P, L, Gain = pend.lqr(Q, R)
A_BK = pend.Ax + np.dot(pend.Bx, Gain)
Ts = pend.Ts

#initial value
theta_0 = 20 #degree
x = np.array( [[theta_0 * math.pi / 180], [0], [0], [0]] )

#total number of the step
num = 1000

#variables
time = []
theta_array = []
theta_array2 = []
theta_dot_array = []
theta_dot_array2 = []
V_array = []
I_array = []

#initialize the lists
time.append(0)
theta_array.append(x[0][0]*180/math.pi) # degree
theta_dot_array.append(x[1][0]*180/math.pi) # degree/sec
theta_array2.append(x[2][0]*r_wheel*100) # cm
theta_dot_array2.append(x[3][0]*r_wheel*100) # cm/sec

#calculate the initial value of motor voltage
Vin = np.dot(Gain, x)[0][0]
if Vin > 3.3:
	Vin = 3.3
if Vin < -3.3:
	Vin = -3.3
V_array.append(Vin)

#calculate the initial value of motor current
I_array.append( (Vin - kb*x[3][0]*gear_ratio)/Rm )

#calculation loop
for i in range(num-1):
	#calculate the next state
	x = np.dot( A_BK, x )

	#angles, angular rates
	theta_array.append(x[0][0] * 180/math.pi) #degree
	theta_dot_array.append(x[1][0] * 180/math.pi) #degree/s
	theta_array2.append(x[2][0] * r_wheel * 100) #cm
	theta_dot_array2.append(x[3][0] * r_wheel * 100) #cm/s

	#motor voltage
	Vin = np.dot(Gain, x)
	if Vin[0][0] > 3.3:
		Vin = [[3.3]]
	if Vin[0][0] < -3.3:
		Vin = [[-3.3]]
	V_array.append( Vin[0][0] )

	#motor current
	I_array.append( (Vin[0][0] - kb*x[3][0]*gear_ratio)/Rm )

	#time
	time.append( Ts*(i+1) )




#===========================================================
#Draw graph
#===========================================================
#create figure object
fig = plt.figure( figsize=(8,8) )

#use "subplot" to divide the graph area
ax1 = fig.add_subplot(6,1,1)
ax2 = fig.add_subplot(6,1,2)
ax3 = fig.add_subplot(6,1,3)
ax4 = fig.add_subplot(6,1,4)
ax5 = fig.add_subplot(6,1,5)
ax6 = fig.add_subplot(6,1,6)

#range of the x axis
stop_time = Ts * num

#angle (degree)
ax1.plot(time, theta_array, color="blue", lw=2)
ax1.set_xlim([0, stop_time])
ax1.set_ylabel("theta (deg)")

#angular rate (degree/sec)
ax2.plot(time, theta_dot_array, color="blue", lw=2)
ax2.set_xlim([0, stop_time])
ax2.set_ylabel("theta_dot (deg/s)")

#position (cm)
ax3.plot(time, theta_array2, color="green", lw=2)
ax3.set_xlim([0, stop_time])
ax3.set_ylabel("x (cm)")

#speed (cm/sec)
ax4.plot(time, theta_dot_array2, color="green", lw=2)
ax4.set_xlim([0, stop_time])
ax4.set_ylabel("x_dot (cm/s)")

#voltage (V)
ax5.plot(time, V_array, color="red", lw=2)
ax5.set_ylim([-4, 4])
ax5.set_xlim([0, stop_time])
ax5.set_ylabel("Voltage (V)")

#current (A)
ax6.plot(time, I_array, color="red", lw=2)
ax6.set_ylim([-2, 2])
ax6.set_xlim([0, stop_time])
ax6.set_ylabel("Current (A)")
ax6.set_xlabel("time (sec)")

#show the graph
plt.tight_layout()
plt.show()
