# -*- coding: utf-8 -*-
"""
Simulation program
"""
from numpy import *
from pylab import *

class SimulationEnvironment:
    def __init__(self, sampling_time):
        self.sampling_time = sampling_time

class IPMSM(SimulationEnvironment):
    def __init__(self,
                 Ld,
                 Lq,
                 ke,
                 resistance,
                 P,
                 sampling_time,
                 control_sampling_time
                 ):
        # plant parameters and simulation conditions
        self.resistance = resistance
        self.Ld = Ld
        self.Lq = Lq
        self.ke = ke
        self.P = P
        self.torque = 0.0
        self.sampling_time = sampling_time
        self.control_sampling_time = control_sampling_time
        self.xvec = [0.0, 0.0]

        # system matrices for control system design

    # calculate derivative of state value
    def calc_deri(self, vd, vq, omega):
        return (-self.resistance / self.Ld * self.xvec[0] + omega * self.Lq / self.Ld * self.xvec[1] + (1.0 / self.Ld) * vd, -omega * self.Ld/self.Lq * self.xvec[0] - self.resistance / self.Lq * self.xvec[1] + (1.0/self.Lq) * (vq - self.ke * omega))
    # update
    def update(self):
        self.xvec[0] = self.xvec[0] + self.dxvec[0] * self.sampling_time
        self.xvec[1] = self.xvec[1] + self.dxvec[1] * self.sampling_time
        self.torque = self.P * self.ke * self.xvec[1] + self.P * (self.Ld - self.Lq) * self.xvec[0] + self.xvec[1]

class SPMSM(SimulationEnvironment):
    def __init__(self,
                 L, #Ld = Lq
                 ke,
                 resistance,
                 P,
                 sampling_time,
                 control_sampling_time
                 ):
        # plant parameters and simulation conditions
        self.resistance = resistance
        self.Ld = L
        self.Lq = L
        self.ke = ke
        self.P = P
        self.torque = 0.0
        self.sampling_time = sampling_time
        self.control_sampling_time = control_sampling_time
        self.xvec = [0.0, 0.0]

        # system matrices for control system design

    # calculate derivative of state value
    def calc_deri(self, vd, vq, omega):
        return [-self.resistance/self.Ld * self.xvec[0] + omega * self.Lq / self.Ld * self.xvec[1] + (1.0/self.Ld) * vd,
                -omega * self.Ld/self.Lq * self.xvec[0] - self.resistance / self.Lq * self.xvec[1] + (1.0/self.Lq) * vq - self.ke * omega]
    # update
    def update(self):
        self.xvec[0] = self.xvec[0] + self.dxvec[0] * self.sampling_time
        self.xvec[1] = self.xvec[1] + self.dxvec[1] * self.sampling_time
        self.torque = self.P * self.ke * xvec[1]

class DCmotor(SimulationEnvironment):
    def __init__(self,
                 resistance,
                 inductance,
                 ke,
                 voltage,
                 sampling_time,
                 control_sampling_time
                 ):
        # plant parameters and simulation conditions
        self.resistance = resistance
        self.inductance = inductance
        self.ke = ke
        self.voltage = voltage
        self.torque = 0.0
        self.sampling_time = sampling_time
        self.control_sampling_time = control_sampling_time
        self.xvec = 0.0

        # for controller

        # system matrices

    # calculate derivative of state value
    def calc_deri(self, voltage, omega):
        return -1.0/self.inductance * self.xvec +  1.0 / self.inductance * voltage - self.ke / self.inductance * omega

    # update
    def update(self):
        self.xvec = self.xvec + self.dxvec * self.sampling_time
        self.torque = self.ke * self.xvec

class rotor(SimulationEnvironment):
    def __init__(self,
                 inertia,
                 friction,
                 sampling_time,
                 control_sampling_time
                 ):
        # plant parameters and simulation conditions
        self.inertia = inertia
        self.friction = friction
        self.sampling_time = sampling_time
        self.control_sampling_time = control_sampling_time
        self.xvec = [0.0, 0.0]

        # for controller

        # system matrices

    # calculate derivative of state value
    def calc_deri(self, torque, torque_reac):
        return (self.xvec[1], -self.friction / self.inertia * self.xvec[1] + 1.0 / self.inertia * torque - (1.0 / self.inertia) * torque_reac)

    # update
    def update(self):
        self.xvec[0] = self.xvec[0] + self.dxvec[0] * self.sampling_time
        self.xvec[1] = self.xvec[1] + self.dxvec[1] * self.sampling_time

# state parameters and data for plot
id_data=[]
iq_data=[]
angle_data=[]
omega_data=[]
angle_cmd_data=[]
omega_cmd_data=[]
id_cmd_data=[]
iq_cmd_data=[]

# simulation object
sim_env = SimulationEnvironment(sampling_time = 0.001)
dc_motor = DCmotor(resistance=1.0, inductance=0.01, ke=0.06, voltage=0.0, sampling_time = sim_env.sampling_time, control_sampling_time=0.001)
IPMSM_motor = IPMSM(Ld=3.9*0.001, Lq=7.9*0.001, ke=47.21*0.001, resistance=154.9*0.001, P=3, sampling_time=sim_env.sampling_time, control_sampling_time=0.001)
SPMSM_motor = SPMSM(L=3.9*0.001, ke=47.21*0.001, resistance=154.9*0.001, P=3, sampling_time=sim_env.sampling_time, control_sampling_time=0.001)
rotor = rotor(inertia = 0.1, friction = 0.001, sampling_time = sim_env.sampling_time, control_sampling_time=0.001)

# main loop 10[sec]
for i in range(3*(int)(1/sim_env.sampling_time)):
    time = i * sim_env.sampling_time

    control_delay = (int)(dc_motor.control_sampling_time/sim_env.sampling_time) #[sample]
    if i%control_delay == 0:
        """ controller """
        # definition for control parameters
        if i == 0 :
            id_err_int = 0.0
            iq_err_int = 0.0

        # position controller (P position feedback control)
        theta_cmd = 1.0
        dtheta_cmd = 20.0 * (theta_cmd - rotor.xvec[0])

        # velocity controller (P velocity feedback control)
        #dtheta_cmd = 50.0
        iq_cmd = 3.0 * (dtheta_cmd - rotor.xvec[1])

        # current controller (PI current feedback control)
        # d current control: Id = 0
        id_cmd = 0.0
        id_err = id_cmd - IPMSM_motor.xvec[0]
        id_err_int = id_err_int + id_err * IPMSM_motor.control_sampling_time
        vd = 5.0 * id_err + 1.5 * id_err_int
        #vd = 0.0
        # q current control
        iq_err = iq_cmd - IPMSM_motor.xvec[1]
        iq_err_int = iq_err_int + iq_err * IPMSM_motor.control_sampling_time
        vq = 5.0 * iq_err + 1.5 * iq_err_int

        #data update
        id_data.append(IPMSM_motor.xvec[0])
        iq_data.append(IPMSM_motor.xvec[1])
        angle_data.append(rotor.xvec[0])
        omega_data.append(rotor.xvec[1])
        angle_cmd_data.append(theta_cmd)
        omega_cmd_data.append(dtheta_cmd)
        id_cmd_data.append(id_cmd)
        iq_cmd_data.append(iq_cmd)
        """ controller end """

    """ plant """
    #reaction torque
    if(time > 1.0):
        torque_reac = 0.1
    else:
        torque_reac = 0.0

    # derivative calculation
    rotor.dxvec = rotor.calc_deri(IPMSM_motor.torque, torque_reac)
    IPMSM_motor.dxvec = IPMSM_motor.calc_deri(vd, vq, rotor.xvec[1])
    # euler-integration
    rotor.update()
    IPMSM_motor.update()

    """ plant end """

# data plot
figure(0)
time_data = np.arange(0, 3, dc_motor.control_sampling_time)
plot(time_data, omega_data[:], label="omega")
legend()
grid()
xlabel('time [s]')
ylabel('omega [rad/sec]')

figure(3)
time_data = np.arange(0, 3, dc_motor.control_sampling_time)
plot(time_data, angle_data[:], label="theta")
legend()
grid()
xlabel('time [s]')
ylabel('theta [rad]')

figure(1)
time_data = np.arange(0, 3, dc_motor.control_sampling_time)
plot(time_data, iq_cmd_data[:], label="iq cmd")
plot(time_data, iq_data[:], label="iq res")
legend()
grid()
xlabel('time [s]')
ylabel('current [A]')

figure(2)
time_data = np.arange(0, 3, dc_motor.control_sampling_time)
plot(time_data, id_cmd_data[:], label="id cmd")
plot(time_data, id_data[:], label="id res")
legend()
grid()
xlabel('time [s]')
ylabel('current [A]')

show()