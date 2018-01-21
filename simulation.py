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
                 Ke,
                 R,
                 P,
                 sampling_time,
                 control_sampling_time
                 ):
        # plant parameters and simulation conditions
        self.R = R
        self.Ld = Ld
        self.Lq = Lq
        self.Ke = Ke
        self.P = P
        self.torque = 0.0
        self.sampling_time = sampling_time
        self.control_sampling_time = control_sampling_time
        self.xvec = [0.0, 0.0]

        # system matrices for control system design

    # calculate derivative of state value
    def calc_deri(self, vd, vq, omega):
        return (-self.R / self.Ld * self.xvec[0] + omega * self.Lq / self.Ld * self.xvec[1] + (1.0 / self.Ld) * vd, -omega * self.Ld/self.Lq * self.xvec[0] - self.R / self.Lq * self.xvec[1] + (1.0/self.Lq) * (vq - self.Ke * omega))
    # update
    def update(self):
        self.xvec[0] = self.xvec[0] + self.dxvec[0] * self.sampling_time
        self.xvec[1] = self.xvec[1] + self.dxvec[1] * self.sampling_time
        self.torque = self.P * self.Ke * self.xvec[1] + self.P * (self.Ld - self.Lq) * self.xvec[0] + self.xvec[1]

class SPMSM(SimulationEnvironment):
    def __init__(self,
                 L, #Ld = Lq
                 Ke,
                 R,
                 P,
                 sampling_time,
                 control_sampling_time
                 ):
        # plant parameters and simulation conditions
        self.R = R
        self.Ld = L
        self.Lq = L
        self.Ke = Ke
        self.P = P
        self.torque = 0.0
        self.sampling_time = sampling_time
        self.control_sampling_time = control_sampling_time
        self.xvec = [0.0, 0.0]

        # system matrices for control system design

    # calculate derivative of state value
    def calc_deri(self, vd, vq, omega):
        return [-self.R/self.Ld * self.xvec[0] + omega * self.Lq / self.Ld * self.xvec[1] + (1.0/self.Ld) * vd,
                -omega * self.Ld/self.Lq * self.xvec[0] - self.R / self.Lq * self.xvec[1] + (1.0/self.Lq) * vq - self.Ke * omega]
    # update
    def update(self):
        self.xvec[0] = self.xvec[0] + self.dxvec[0] * self.sampling_time
        self.xvec[1] = self.xvec[1] + self.dxvec[1] * self.sampling_time
        self.torque = self.P * self.Ke * self.xvec[1]

class DCmotor(SimulationEnvironment):
    def __init__(self,
                 Rm,
                 Lm,
                 Ke,
                 voltage,
                 sampling_time,
                 control_sampling_time): # plant parameters and simulation conditions
        self.Rm = Rm
        self.Lm = Lm
        self.Ke = Ke
        self.voltage = voltage
        self.torque = 0.0
        self.sampling_time = sampling_time
        self.control_sampling_time = control_sampling_time
        self.xvec = 0.0

        # for controller

        # system matrices

    # calculate derivative of state value
    def calc_deri(self, voltage, omega):
        return -self.Rm/self.Lm * self.xvec +  1.0 / self.Lm * voltage - self.Ke / self.Lm * omega

    # update
    def update(self):
        self.xvec = self.xvec + self.dxvec * self.sampling_time
        self.torque = self.Ke * self.xvec

class RigidRotor(SimulationEnvironment):
    def __init__(self,
                 J,
                 B,
                 sampling_time,
                 control_sampling_time
                 ):
        # plant parameters and simulation conditions
        self.J = J
        self.B = B
        self.sampling_time = sampling_time
        self.control_sampling_time = control_sampling_time
        self.xvec = [0.0, 0.0]

        # for controller

        # system matrices
# calculate derivative of state value
    def calc_deri(self, torque, torque_reac):
        return (self.xvec[1], -self.B / self.J * self.xvec[1] + 1.0 / self.J * torque - (1.0 / self.J) * torque_reac)

    # update
    def update(self):
        self.xvec[0] = self.xvec[0] + self.dxvec[0] * self.sampling_time
        self.xvec[1] = self.xvec[1] + self.dxvec[1] * self.sampling_time

class TwoInertiaRotor(SimulationEnvironment):
    def __init__(self,
                 Jm,
                 Bm,
                 Jl,
                 Bl,
                 Ks,
                 Ds,
                 sampling_time,
                 control_sampling_time
                 ):
        # plant parameters and simulation conditions
        self.Jm = Jm
        self.Bm = Bm
        self.Jl = Jl
        self.Bl = Bl
        self.Ks = Ks
        self.Ds = Ds
        self.sampling_time = sampling_time
        self.control_sampling_time = control_sampling_time
        self.xvec = [0.0, 0.0, 0.0, 0.0]

        # for controller

        # system matrices

    # calculate derivative of state value
    def calc_deri(self, taum, taum_dis, taul_dis):
        return (self.xvec[1],
                 - self.Bm / self.Jm * self.xvec[1] - self.Ks / self.Jm * self.xvec[0] - self.Ds / self.Jm * self.xvec[1] + self.Ks / self.Jm * self.xvec[2] + self.Ds / self.Jm * self.xvec[3] + taum / self.Jm - taum_dis / self.Jm,
                self.xvec[3],
                - self.Bl / self.Jl * self.xvec[3] + self.Ks / self.Jl * self.xvec[0] + self.Ds / self.Jl * self.xvec[1] - self.Ks / self.Jl * self.xvec[2]  - self.Ds / self.Jl * self.xvec[3] - taul_dis / self.Jl
                )

    # update
    def update(self):
        self.xvec[0] = self.xvec[0] + self.dxvec[0] * self.sampling_time
        self.xvec[1] = self.xvec[1] + self.dxvec[1] * self.sampling_time
        self.xvec[2] = self.xvec[2] + self.dxvec[2] * self.sampling_time
        self.xvec[3] = self.xvec[3] + self.dxvec[3] * self.sampling_time
