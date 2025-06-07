import numpy as np
import math
import control
import os
import json


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
        
        # sampling time
        self.Ts = 0.01  # [s]
        
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
        self.gear_ratio = 50

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
        
        # 車輪回転軸の粘性摩擦係数
        self.c = self.gear_ratio**2 * self.kt * self.kb / self.Rm

        # LQRパラメータ
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
        print("The length between the center of gravity and the axis r_pendulum (m)")
        print("r_pendulum = " + str(self.r_pendulum) + " (m)")
        print("I_pendulum = " + str(self.I_pendulum) + " (kg.m^2)\n")
        
        print("------------------------------")
        print("Rotator Parameters")
        print("m_rotator = " + str(self.m_rotator) + " (kg)")
        print("r_rotator = " + str(self.r_rotator) + " (m)")
        print("Im = " + str(self.Im) + " (kg.m^2)\n")

        print("------------------------------")


    def calc_continuous_system(self):
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
        
        d1 = (m_whole * r_wheel ** 2 
                + I_wheel + gear_ratio ** 2 * Im)
        
        d2 = (m_whole * r_wheel ** 2
                + m_pendulum * r_wheel * r_pendulum + I_wheel + Im * gear_ratio)
        
        d3 = d2
        
        d4 = (m_whole * r_wheel ** 2 
                + 2 * m_pendulum * r_wheel * r_pendulum 
                + m_pendulum * r_pendulum ** 2
                + I_pendulum + I_wheel + Im)
        
        # print("Inverted Pendulum Parameters")
        # print("d1 = " + str(d1))
        # print("d2 = " + str(d2))
        # print("d3 = " + str(d3))
        # print("d4 = " + str(d4))
        
        det = d2 * d3 - d1 * d4
        a11 = (-d1 * m_pendulum * g * r_pendulum) / det
        a12 = (-d3 * gear_ratio ** 2 * kt * kb / Rm) / det
        a41 = (d2 * m_pendulum * g * r_pendulum) / det
        a44 = (d4 * gear_ratio ** 2 * kt * kb / Rm) / det
        
        b2  = (d3 * gear_ratio * kt / Rm) / det
        b4  = (d4 * gear_ratio * kt / Rm) / det
        
        # continuous time
        A = [[0, 1, 0, 0],
             [a11, 0, 0, a12],
             [0, 0, 0, 1],
             [a41, 0, 0, a44]]
        
        B = [[0],
             [b2],
             [0],
             [b4]]
        
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
        self.calc_continuous_system()
        
        # discrete
        Ts = self.Ts
        A = self.A
        B = self.B
        C = self.C
        
        #Ad = I + AT + (AT)^2/2! + ... (5th order approximation)
        Ad = np.zeros( (len(A), len(A[0])) )
        temp = np.eye(len(A))
        for i in range(10):
            Ad = Ad + temp / math.factorial(i)
            temp = np.dot(temp, (A*Ts))
        
        #Bd = {IT + AT^2/2! + A^2T^3/3! + ...}B (5th order approximation)
        Bx_temp = np.eye(len(A)) * Ts
        temp = A * Ts * Ts
        for i in range(10):
            Bx_temp = Bx_temp + temp / math.factorial(i+2)
            temp = np.dot(temp, A*Ts)
        Bd = np.dot(Bx_temp, B)
        
        # Cd = same as continuous time
        Cd = C
        
        self.Ad = Ad
        self.Bd = Bd
        self.Cd = Cd
        self.Ts = Ts

        print("******************************")
        print("*        Ac, Bc, Cc          *")
        print("******************************")
        print("System Matrix")
        print("A = \n" + str(self.A))
        print("B = \n" + str(self.B))
        print("C = \n" + str(self.C))
        
        print("******************************")
        print("*    Pole of the system      *")
        print("******************************")
        print("Eigenvalue of A = \n" + str(self.eig(self.A)[0]))
        
        print("******************************")
        print("*        Ad, Bd, Cd, Ts          *")
        print("******************************")
        print("System Matrix")
        print("Ad = \n" + str(self.Ad))
        print("Bd = \n" + str(self.Bd))
        print("Cd = \n" + str(self.Cd))
        print("Ts = \n" + str(self.Ts))
    
    def lqr(self, Q, R):
        # 
        # return:
        # P: solution of the Riccati equation
        # L: eigenvalues
        # G: gain
                
        P, L, G = control.care(self.A, self.B, Q, R)
        return P, L, -G
    
    def dlqr(self, Q, R):
        # 
        # return:
        # P: solution of the Riccati equation
        # L: eigenvalues
        # G: gain
                
        Qd = Q * self.Ts
        Rd = R * self.Ts
        P, L, G = control.dare(self.Ad, self.Bd, Qd, Rd)
        return P, L, -G
        
    def eig(self, A):
        eigen_value, eigen_vector = np.linalg.eig(A)
        return eigen_value, eigen_vector
    
    def c2d_poles(self, pole_c):
        # Convert continuous poles "s" to discrete poles "z"
        # z = e^(sT)
        # s = ln(z)/T
        pole_d = [np.exp(pole * self.Ts) for pole in pole_c]
        return pole_d
    
    def d2c_poles(self, pole_d):
        # Convert discrete poles "z" to continuous poles "s"
        # s = ln(z)/T
        # z = e^(pole * Ts)
        pole_c = [np.log(pole) / self.Ts for pole in pole_d]
        return pole_c
    
    def continuous_acker(self, pole):
        # Calculate the feedback gain using Ackermann's Pole Placement Method
        G = -control.acker(self.A, self.B, pole)
        return G
    
    def discrete_acker(self, pole):
        # Calculate the feedback gain using Ackermann's Pole Placement Method
        G = -control.acker(self.Ad, self.Bd, pole)
        return G
    
    def output_gain(self, file_path, gain_data):
        # JSON ファイルに保存
        dir_path = os.path.dirname(file_path)
        os.makedirs(dir_path, exist_ok=True)
        with open(file_path, "w") as f:
            json.dump(gain_data, f, indent=4, separators=(",", ": "))

    def array_to_cpp(self, matrix, matrix_name):
        rows, cols = len(matrix), len(matrix[0])
        
        # データ型と変数宣言の作成
        cpp_declaration = f"float {matrix_name}[{rows}][{cols}] = {{"
        
        for i in range(rows):
            if i == 0:
                cpp_declaration += "\n    {"
            else:
                cpp_declaration += ",\n    {"
                
            for j in range(cols):
                cpp_declaration += str(matrix[i][j])
                if j < cols - 1:
                    cpp_declaration += ", "
            cpp_declaration += "}"
        
        cpp_declaration += "};"
        return cpp_declaration

    def print_array_to_cpp(self):
        # Print the matrices in C++ format with proper variable names and comments
        print("// matrix A_x")
        print(self.array_to_cpp(self.Ad, "A_x"))
        print()
        print("// matrix B_x") 
        print(self.array_to_cpp(self.Bd, "B_x"))
        print()
        print("// matrix C_x")
        print(self.array_to_cpp(self.Cd, "C_x"))