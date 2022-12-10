import sys
sys.path.append('../')

import numpy as np
from simulation.simulation import *

def IPMSM_positioning(simulation_time, plant_parameters, control_parameters, external_inputs):

    # state parameters and data for plot
    id_data=[]
    iq_data=[]
    qm_data=[]
    dqm_data=[]
    qm_cmd_data=[]
    dqm_cmd_data=[]
    dqm_ref_data=[]
    id_cmd_data=[]
    iq_ref_data=[]
    qm_err_data=[]
    dqm_err_data=[]
    id_err_data=[]
    iq_err_data=[]
    time_data=[]

    sampling_time=plant_parameters[0]
    J=plant_parameters[1]
    B=plant_parameters[2]
    R=plant_parameters[3]
    Ld=plant_parameters[4]
    Lq=plant_parameters[5]
    Ke=plant_parameters[6]
    P=plant_parameters[7]

    Kpp = control_parameters[0]
    Kpv = control_parameters[1]
    Kiv = control_parameters[2]
    Kpi = control_parameters[3]
    Kii = control_parameters[4]

    qm_cmd = external_inputs[0]
    dqm_cmd = external_inputs[1]
    torque_reac = external_inputs[2]

    # simulation object
    sim_env = SimulationEnvironment(sampling_time = sampling_time)
    IPMSM_motor = IPMSM(Ld=Ld, Lq=Lq, Ke=Ke, R=R, P=P, sampling_time=sim_env.sampling_time, control_sampling_time=sampling_time)
    rigid_rotor = RigidRotor(J=J, B=B, sampling_time = sim_env.sampling_time, control_sampling_time=sampling_time)

    # main loop 10[sec]
    for i in range(int(simulation_time*(1/sim_env.sampling_time))):
        time = i * sim_env.sampling_time

        control_delay = (int)(IPMSM_motor.control_sampling_time/sim_env.sampling_time) #[sample]
        if i%control_delay == 0:
            """ controller """
            # definition for control parameters
            if i == 0 :
                id_err_int = 0.0
                iq_err_int = 0.0

            # position controller (P position feedback control)
            qm_err = qm_cmd[i] - rigid_rotor.xvec[0]
            dqm_ref = Kpp * qm_err

            # velocity controller (P velocity feedback control)
            dqm_err = dqm_ref - rigid_rotor.xvec[1]
            iq_ref = Kpv * dqm_err # + dqm_cmd[i]

            # current controller (PI current feedback control)
            # d current control: Id = 0
            id_cmd = 0.0
            id_err = id_cmd - IPMSM_motor.xvec[0]
            id_err_int = id_err_int + id_err * IPMSM_motor.control_sampling_time
            vd = Kpi * id_err + Kii * id_err_int
            # q current control
            iq_err = iq_ref - IPMSM_motor.xvec[1]
            iq_err_int = iq_err_int + iq_err * IPMSM_motor.control_sampling_time
            vq = Kpi * iq_err + Kii * iq_err_int

            #data update
            time_data.append(time)
            id_data.append(IPMSM_motor.xvec[0])
            iq_data.append(IPMSM_motor.xvec[1])
            qm_data.append(rigid_rotor.xvec[0])
            dqm_data.append(rigid_rotor.xvec[1])
            qm_cmd_data.append(qm_cmd[i])
            dqm_cmd_data.append(dqm_cmd[i])
            dqm_ref_data.append(dqm_ref)
            id_cmd_data.append(id_cmd)
            iq_ref_data.append(iq_ref)
            qm_err_data.append(qm_err)
            dqm_err_data.append(dqm_err)
            id_err_data.append(id_err)
            iq_err_data.append(iq_err)

            """ controller end """

        """ plant """

        # derivative calculation
        rigid_rotor.dxvec = rigid_rotor.calc_deri(IPMSM_motor.torque, torque_reac[i])
        IPMSM_motor.dxvec = IPMSM_motor.calc_deri(vd, vq, rigid_rotor.xvec[1])
        # euler-integration
        rigid_rotor.update()
        IPMSM_motor.update()

        """ plant end """

    # data plot
    from matplotlib import pyplot as plt
    plt.figure(figsize=(10, 7))
    plt.subplot(421)
    plt.plot(time_data, qm_cmd_data, label="theta motor cmd")
    plt.plot(time_data, qm_data, label="theta motor res")
    plt.legend()
    plt.grid()
    plt.ylabel('theta [rad]')

    plt.subplot(423)
    plt.plot(time_data, dqm_ref_data, label="omega motor cmd")
    plt.plot(time_data, dqm_data, label="omega motor res")
    plt.legend()
    plt.grid()
    plt.ylabel('omega [rad/s]')

    plt.subplot(425)
    plt.plot(time_data, id_cmd_data, label="id cmd")
    plt.plot(time_data, id_data, label="id res")
    plt.legend()
    plt.grid()
    plt.ylabel('current [A]')

    plt.subplot(427)
    plt.plot(time_data, iq_ref_data, label="iq cmd")
    plt.plot(time_data, iq_data, label="iq res")
    plt.legend()
    plt.grid()
    plt.ylabel('current [A]')

    plt.subplot(422)
    plt.plot(time_data, qm_err_data, label="theta error")
    plt.legend()
    plt.grid()

    plt.subplot(424)
    plt.plot(time_data, dqm_err_data, label="omega error")
    plt.legend()
    plt.grid()

    plt.subplot(426)
    plt.plot(time_data, id_err_data, label="id error")
    plt.legend()
    plt.grid()

    plt.subplot(428)
    plt.plot(time_data, iq_err_data, label="iq error")
    plt.legend()
    plt.grid()

    plt.show()


if __name__ == "__main__":

    simulation_time = 3.0
    sampling_time = 0.0001 # 100 us

    #################################################################
    ### example 2 IPMSM positioning
    #################################################################
    Ld=3.9*0.001
    Lq=7.9*0.001
    Ke=47.21*0.001
    R=154.9*0.001
    P=3
    J=0.1
    B=0.001
    plant_parameters = [sampling_time, J, B, R, Ld, Lq, Ke, P]

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
    Kpi = 5.0                           # P gain for current control loop
    Kii = 1.5                          # I gain for current control loop
    control_parameters = [Kpp, Kpv, Kiv, Kpi, Kii]

    # Simulation
    IPMSM_positioning(simulation_time, plant_parameters, control_parameters, external_inputs)
