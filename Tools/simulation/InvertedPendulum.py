import numpy as np
import math
import control


class InvertedPendulum:
    # m_*: mass [kg]
    # r_*: radius [m]
    # x_*: length of x direction [m]
    # y_*: length of y direction [m]
    # I_*: moment of inertia [kg.m^2]
    # d_*: The length between the center of gravity and the axis [m]
    
    def __init__(self):
        # gravitational acceleration
        self.g = 9.8    # [m/s^2]
        
        # wheel (Tamiya sports tire set)
        self.m_wheel = 0.026
        self.r_wheel = 0.028
        self.I_wheel = 0.5 * self.m_wheel * self.r_wheel ** 2

        # Gear box (Tamiya high power gear box HE)
        self.m_gear = 0.058
        self.x_gear = 0.028
        self.y_gear = 0.060
        self.d_gear = 0.020
        self.I_gear = (1/12) * self.m_gear * (self.x_gear**2 + self.y_gear**2) + self.m_gear * self.d_gear**2
        self.gear_ratio = 64.8

        # Chassis (Tamiya universal plate L)
        self.m_plate = 0.080 / 2
        self.y_plate = 0.210
        self.d_plate = 0.095
        self.I_plate = (1/12) * self.m_plate * self.y_plate**2 + self.m_plate * self.d_plate**2

        # Battery box (AA size x 4)
        self.m_battery = 0.120 / 2
        self.x_battery = 0.015
        self.y_battery = 0.056
        self.d_battery = 0.065
        self.I_battery = (1/12) * self.m_battery * (self.x_battery**2 + self.y_battery**2) + self.m_battery * self.d_battery**2

        # Circuit board (Raspberry Pi)
        self.m_circuit = 0.100 / 2
        self.x_circuit = 0.010
        self.y_circuit = 0.095
        self.d_circuit = 0.140
        self.I_circuit = (1/12) * self.m_circuit * (self.x_circuit**2 + self.y_circuit**2) + self.m_circuit * self.d_circuit**2

        # Whole body
        self.m_pendulum = self.m_gear + self.m_battery + self.m_plate + self.m_circuit
        self.I_pendulum = self.I_gear + self.I_battery + self.I_plate + self.I_circuit
        # The length between the center of gravity and the axis (m)
        self.r_pendulum = math.sqrt(self.I_pendulum / self.m_pendulum)
        
        # Whole system
        self.m_whole = self.m_pendulum + self.m_wheel
        self.I_whole = self.I_pendulum + self.I_wheel

        # Motor (RE-260RA-2670)
        self.Rm = 2.4           # resistance
        self.kt = 0.0018        # torque constant [N.m/A]
        self.kb = 0.0024        # back electromotive force constant [V.s/rad]
        self.m_rotator = 0.010
        self.r_rotator = 0.0075
        # moment of inertia of the rotator
        self.Im = 0.5 * self.m_rotator * self.r_rotator ** 2

        # LQR パラメータ
        self.Q = np.diag([10, 10, 10, 10])
        self.R = 100.0
        
        #-------------------------------------------------
        # parameter summary
        #-------------------------------------------------
        print("******************************")
        print("*        Parameters          *")
        print("******************************")
        print("Wheel Parameters")
        print("m_wheel = " + str(self.m_wheel) + " (kg)")
        print("r_wheel = " + str(self.r_wheel) + " (m)")
        print("I_wheel = " + str(self.I_wheel) + " (kg.m^2)\n")

        print("------------------------------")
        print("Pendulum Parameters")
        print("m_pendulum = " + str(self.m_pendulum) + " (kg)")
        print("r_pendulum = " + str(self.r_pendulum) + " (m)")
        print("I_pendulum = " + str(self.I_pendulum) + " (kg.m^2)\n")



    def calc_continous_system(self):
        g = self.g
        m_whole = self.m_whole
        
        m_pendulum = self.m_pendulum
        r_pendulum = self.r_pendulum
        I_pendulum = self.I_pendulum
        
        r_wheel = self.r_wheel
        I_wheel = self.I_wheel
        
        Im = self.Im
        gear_ratio = self.gear_ratio
        kt = self.kt
        kb = self.kb
        Rm = self.Rm
        
        a11_temp = (m_whole * r_wheel ** 2 
                    + 2 * m_pendulum * r_wheel * r_pendulum 
                    + m_pendulum * r_pendulum ** 2
                    + I_pendulum + I_wheel)
        
        a12_temp = (m_whole * r_wheel ** 2
                    + m_pendulum * r_wheel * r_pendulum + I_wheel)
        
        a21_temp = (m_whole * r_wheel ** 2 
                    + m_pendulum * r_wheel * r_pendulum + I_wheel)

        a22_temp = (m_whole * r_wheel ** 2 
                    + I_wheel + gear_ratio ** 2 * Im)
        
        det = a11_temp*a22_temp - a12_temp*a21_temp
        a11 = (a22_temp * m_pendulum * g * r_pendulum) / det
        a12 = (a12_temp * gear_ratio ** 2 * kt * kb / Rm) / det
        a21 = (-a21_temp * m_pendulum * g * r_pendulum) / det
        a22 = (-a11_temp * gear_ratio ** 2 * kt * kb / Rm) / det
        
        b21  = (-a12_temp * gear_ratio * kt / Rm) / det
        b24  = (a11_temp * gear_ratio * kt / Rm) / det
        
        # continuous time
        A = [[0, 1, 0, 0],
             [a11, 0, 0, a12],
             [0, 0, 0, 1],
             [a21, 0, 0, a22]]
        
        B = [[0],
             [b21],
             [0],
             [b24]]
        
        C = [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]]
        
        # continuous time
        self.A = np.array(A)
        self.B = np.array(B)
        self.C = np.array(C)

        return self.A, self.B, self.C
        
        
    def calc_discrete_system(self):
        # discrete
        Ts = 0.01 # [s]
        A = self.A
        B = self.B
        
        #Ax = I + AT + (AT)^2/2! + ... (5th order approximation)
        Ax = np.zeros( (len(A), len(A[0])) )
        temp = np.eye(len(A))
        for i in range(10):
        	Ax = Ax + temp / math.factorial(i)
        	temp = np.dot(temp, (A*Ts))
        
        #Bx = {IT + AT^2/2! + A^2T^3/3! + ...}B (5th order approximation)
        Bx_temp = np.eye(len(A)) * Ts
        temp = A * Ts * Ts
        for i in range(10):
            Bx_temp = Bx_temp + temp / math.factorial(i+2)
            temp = np.dot(temp, A*Ts)
        Bx = np.dot(Bx_temp, B)
        
        # Cx = same as continuous time
        Cx = C
        
        self.Ax = Ax
        self.Bx = Bx
        self.Cx = Cx
        self.Ts = Ts
        
    def lqr(self, Q, R):
        # 
        # return:
        # P: solution of the Riccati equation
        # L: eigenvalues
        # G: gain
                
        Qd = Q * self.Ts
        Rd = R * self.Ts
        P, L, G = control.dare(self.Ax, self.Bx, Qd, Rd)
        return P, L, -G
        
    def eig(self, A):
        eigen_value, eigen_vector = np.linalg.eig(A)
        return eigen_value, eigen_vector
    
    def acker(self, pole):
        # Calculate the feedback gain using Ackermann's Pole Placement Method
        G = -control.acker(self.Ax, self.Bx, pole)
        return G