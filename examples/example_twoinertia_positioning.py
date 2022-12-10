import sys
sys.path.append('../')

import numpy as np
from simulation.simulation import *

def two_inertia_positioning(simulation_time, plant_parameters, control_parameters, external_inputs):

    # state parameters and data for plot
    qm_data=[]
    dqm_data=[]
    qm_cmd_data=[]
    dqm_cmd_data=[]
    dqm_ref_data=[]
    ql_data=[]
    dql_data=[]
    qm_err_data=[]
    dqm_err_data=[]
    ql_err_data=[]
    dql_err_data=[]
    time_data=[]

    sampling_time=plant_parameters[0]
    Jm=plant_parameters[1]
    Bm=plant_parameters[2]
    Jl=plant_parameters[3]
    Bl=plant_parameters[4]
    Ks=plant_parameters[5]
    Ds=plant_parameters[6]

    Kp = control_parameters[0]
    Kv = control_parameters[1]
    Ti = control_parameters[2]

    qm_cmd = external_inputs[0]
    dqm_cmd = external_inputs[1]
    torque_reac = external_inputs[2]

    # simulation object
    sim_env = SimulationEnvironment(sampling_time=sampling_time)
    springy_rotor = TwoInertiaRotor(Jm=Jm,
                                    Bm=Bm,
                                    Jl=Jl,
                                    Bl=Bl,
                                    Ks=Ks,
                                    Ds=Ds,
                                    sampling_time = sim_env.sampling_time,
                                    control_sampling_time=sim_env.sampling_time)

    # main loop
    for i in range(int(simulation_time*(1/sim_env.sampling_time))):
        time = i * sim_env.sampling_time

        control_delay = (int)(springy_rotor.control_sampling_time/sim_env.sampling_time) #[sample]
        if i%control_delay == 0:
            """ controller """
            # definition for control parameters
            if i == 0 :
                dqm_err_int = 0.0

            # position controller (P position feedback control)
            qm_err = qm_cmd[i] - springy_rotor.xvec[0]
            dqm_ref = Kp * qm_err # + dqm_cmd[i]

            # velocity controller (PI velocity feedback control)
            dqm_err = dqm_ref - springy_rotor.xvec[1]
            dqm_err_int = dqm_err_int + springy_rotor.control_sampling_time * dqm_err
            torque_cmd = Kv * (dqm_err + dqm_err_int / Ti)

            #data update
            time_data.append(time)
            qm_data.append(springy_rotor.xvec[0])
            dqm_data.append(springy_rotor.xvec[1])
            qm_cmd_data.append(qm_cmd[i])
            dqm_cmd_data.append(dqm_cmd[i])
            dqm_ref_data.append(dqm_ref)
            ql_data.append(springy_rotor.xvec[2])
            dql_data.append(springy_rotor.xvec[3])
            qm_err_data.append(qm_err)
            dqm_err_data.append(dqm_err)
            ql_err_data.append(qm_cmd[i] - springy_rotor.xvec[2])
            dql_err_data.append(dqm_ref - springy_rotor.xvec[3])
            """ controller end """

        """ plant """

        # derivative calculation
        springy_rotor.dxvec = springy_rotor.calc_deri(torque_cmd, 0.0, torque_reac[i])
        # euler-integration
        springy_rotor.update()

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
    plt.plot(time_data, qm_cmd_data, label="theta link cmd")
    plt.plot(time_data, ql_data, label="theta link res")
    plt.legend()
    plt.grid()
    plt.ylabel('theta [rad]')

    plt.subplot(427)
    plt.plot(time_data, dqm_ref_data, label="omega link cmd")
    plt.plot(time_data, dql_data, label="omega link res")
    plt.legend()
    plt.grid()
    plt.xlabel('time [s]')
    plt.ylabel('theta [rad]')

    plt.subplot(422)
    plt.plot(time_data, qm_err_data, label="theta motor error")
    plt.legend()
    plt.grid()

    plt.subplot(424)
    plt.plot(time_data, dqm_err_data, label="omega motor error")
    plt.legend()
    plt.grid()

    plt.subplot(426)
    plt.plot(time_data, ql_err_data, label="theta link error")
    plt.legend()
    plt.grid()

    plt.subplot(428)
    plt.plot(time_data, dql_err_data, label="omega link error")
    plt.legend()
    plt.grid()
    plt.xlabel('time [s]')

    plt.show()


if __name__ == "__main__":

    simulation_time = 3.0
    sampling_time = 0.0001 # 100 us

    #################################################################
    ### example 4 two-inertia resonant positioning
    #################################################################
    Jm=0.1    # motor inertia [kgm^2]
    Bm=0.01    # motor viscous friction [Nm/rad^s]
    Jl=0.1    # link inertia [kgm^2]
    Bl=0.01    # link viscous friction [Nm/s]
    Ks=100.0  # coupling spring [Nm/rad]
    Ds=1.0    # coupling dampling [Nm/rads]
    plant_parameters = [sampling_time, Jm, Bm, Jl, Bl, Ks, Ds]
    fr = np.sqrt((Ks/Jl) * (1 + Jl / Jm)) / (2.0*np.pi)
    far = np.sqrt(Ks/Jl) / (2.0*np.pi)
    print('Resonant frequency {0:6.2f}'.format(fr))
    print('Anti-resonant frequency {0:6.2f}'.format(far))

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
    Kv = 150.0                         # P gain for velocity control loop
    Kp = 5.0                           # P gain for position control loop
    Ki = 10.0                          # I gain for velocity control loop
    control_parameters = [Kp, Kv, Ki]

    # Simulation
    two_inertia_positioning(simulation_time, plant_parameters, control_parameters, external_inputs)
