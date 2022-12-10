import sys
sys.path.append('../')

import numpy as np
from simulation.simulation import *

def DCmotor_positioning(simulation_time, plant_parameters, control_parameters, external_inputs):

    # state parameters and data for plot
    i_data=[]
    qm_data=[]
    dqm_data=[]
    qm_cmd_data=[]
    dqm_cmd_data=[]
    dqm_ref_data=[]
    i_cmd_data=[]
    qm_err_data=[]
    dqm_err_data=[]
    i_err_data=[]
    time_data=[]

    sampling_time=plant_parameters[0]
    Jm=plant_parameters[1]
    Bm=plant_parameters[2]
    Rm=plant_parameters[3]
    Lm=plant_parameters[4]
    Ke=plant_parameters[5]

    Kpp = control_parameters[0]
    Kpv = control_parameters[1]
    Kiv = control_parameters[2]
    Kpi = control_parameters[3]
    Kii = control_parameters[4]

    qm_cmd = external_inputs[0]
    dqm_cmd = external_inputs[1]
    torque_reac = external_inputs[2]

    # simulation object
    sim_env = SimulationEnvironment(sampling_time=sampling_time)
    dc_motor = DCmotor(Rm=Rm, Lm=Lm, Ke=Ke, voltage=0.0, sampling_time = sim_env.sampling_time, control_sampling_time=sampling_time)
    rigid_rotor = RigidRotor(J=Jm, B=Bm, sampling_time = sim_env.sampling_time, control_sampling_time=sim_env.sampling_time)

    # main loop
    for i in range(int(simulation_time*(1/sim_env.sampling_time))):
        time = i * sim_env.sampling_time

        control_delay = (int)(dc_motor.control_sampling_time/sim_env.sampling_time) #[sample]
        if i%control_delay == 0:
            """ controller """
            # definition for control parameters
            if i == 0 :
                i_err_int = 0.0
                dqm_err_int = 0.0

            # position controller (P position feedback control)
            qm_err = qm_cmd[i] - rigid_rotor.xvec[0]
            dqm_ref = Kpp * qm_err

            # velocity controller (P velocity feedback control)
            dqm_err = dqm_ref - rigid_rotor.xvec[1]
            dqm_err_int = dqm_err_int + dqm_err * dc_motor.control_sampling_time
            i_ref = Kpv * dqm_err + Kiv * dqm_err_int # + dqm_cmd[i]

            # current controller (PI current feedback control)
            i_err = i_ref - dc_motor.xvec
            i_err_int = i_err_int + i_err * dc_motor.control_sampling_time
            v_ref = Kpi * i_err + Kii * i_err_int

            #data update
            time_data.append(time)
            i_data.append(dc_motor.xvec)
            qm_data.append(rigid_rotor.xvec[0])
            dqm_data.append(rigid_rotor.xvec[1])
            qm_cmd_data.append(qm_cmd[i])
            dqm_cmd_data.append(dqm_cmd[i])
            dqm_ref_data.append(dqm_ref)
            i_cmd_data.append(i_ref)
            qm_err_data.append(qm_err)
            dqm_err_data.append(dqm_err)
            i_err_data.append(i_err)

            """ controller end """

        """ plant """

        # derivative calculation
        rigid_rotor.dxvec = rigid_rotor.calc_deri(dc_motor.torque, torque_reac[i])
        dc_motor.dxvec = dc_motor.calc_deri(v_ref, rigid_rotor.xvec[1])
        # euler-integration
        rigid_rotor.update()
        dc_motor.update()

        """ plant end """

    # data plot
    from matplotlib import pyplot as plt
    plt.figure(figsize=(10, 7))
    plt.subplot(321)
    plt.plot(time_data, qm_cmd_data, label="theta motor cmd")
    plt.plot(time_data, qm_data, label="theta motor res")
    plt.legend()
    plt.grid()
    plt.ylabel('theta [rad]')

    plt.subplot(323)
    plt.plot(time_data, dqm_ref_data, label="omega motor cmd")
    plt.plot(time_data, dqm_data, label="omega motor res")
    plt.legend()
    plt.grid()
    plt.ylabel('omega [rad/s]')

    plt.subplot(325)
    plt.plot(time_data, i_cmd_data, label="motor current cmd")
    plt.plot(time_data, i_data, label="motor current res")
    plt.legend()
    plt.grid()
    plt.ylabel('current [A]')

    plt.subplot(322)
    plt.plot(time_data, qm_err_data, label="theta error")
    plt.legend()
    plt.grid()

    plt.subplot(324)
    plt.plot(time_data, dqm_err_data, label="omega error")
    plt.legend()
    plt.grid()

    plt.subplot(326)
    plt.plot(time_data, i_err_data, label="motor error")
    plt.legend()
    plt.grid()

    plt.show()



if __name__ == "__main__":

    simulation_time = 3.0
    sampling_time = 0.0001 # 100 us

    #################################################################
    ### example 3 DC motor positioning
    #################################################################
    Jm=0.1       # motor inertia [kgm^2]
    Bm=0.0       # motor viscous friction [Nm/rad^s]
    Rm = 0.1     # resistance [ohm]
    Lm = 0.1     # inductance [H]
    Ke = 50.0*0.001   # torque constant [Nm/A]
    plant_parameters = [sampling_time, Jm, Bm, Rm, Lm, Ke]

    # Command
    qm_cmd = []
    dqm_cmd = []
    for i in range(int(simulation_time/sampling_time)):
        time = i * sampling_time
        if time <= 0.5:
            qm_cmd_tmp = 0.0
            dqm_cmd_tmp = 0.0
        else:
            qm_cmd_tmp = 1.0
            dqm_cmd_tmp = 0.0
        qm_cmd.append(qm_cmd_tmp)
        dqm_cmd.append(dqm_cmd_tmp)

    # Disturbance
    torque_reac = []
    for i in range(int(simulation_time/sampling_time)):
        time = i * sampling_time
        torque_reac_tmp = 0.0
        torque_reac.append(torque_reac_tmp)

    external_inputs = [qm_cmd, dqm_cmd, torque_reac]

    # Position gains
    Kpp = 2.0                         # P gain for velocity control loop
    Kpv = 10.0                           # P gain for velocity control loop
    Kiv = 0.5                          # I gain for velocity control loop
    Kpi = 1.0                           # P gain for current control loop
    Kii = 1.0                          # I gain for current control loop
    control_parameters = [Kpp, Kpv, Kiv, Kpi, Kii]

    # Simulation
    DCmotor_positioning(simulation_time, plant_parameters, control_parameters, external_inputs)
