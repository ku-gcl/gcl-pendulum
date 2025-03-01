
class InvertedPendulum:
    # m_*: mass [kg]
    # r_*: radius [m]
    # x_*: length of x direction [m]
    # y_*: length of y direction [m]
    # I_*: moment of inertia [kg.m^2]
    # d_*: The length between the center of gravity and the axis [m]
    
    def __init__(self):
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

