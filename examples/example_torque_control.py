import numpy as np
from simulation import *

def two_inertia_torque_control(simulation_time, plant_parameters, control_parameters, external_inputs):

    # state parameters and data for plot
    qm_data=[]
    dqm_data=[]
    ql_data=[]
    dql_data=[]
    torque_cmd_data=[]
    dtorque_cmd_data=[]
    torque_data=[]
    dtorque_data=[]
    torque_err_data=[]
    dtorque_err_data=[]
    time_data=[]

    sampling_time=plant_parameters[0]
    Jm=plant_parameters[1]
    Bm=plant_parameters[2]
    Jl=plant_parameters[3]
    Bl=plant_parameters[4]
    Ks=plant_parameters[5]
    Ds=plant_parameters[6]

    Kt = control_parameters[0]
    Ks = control_parameters[1]
    wt = control_parameters[2]

    torque_cmd = external_inputs[0]
    dtorque_cmd = external_inputs[1]
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
                pass

            # torque controller (P position feedback control)
            torque_res =  Ks * (springy_rotor.xvec[0] - springy_rotor.xvec[2])
            dtorque_res =  Ks * (springy_rotor.xvec[1] - springy_rotor.xvec[3])
            torque_err = torque_cmd[i] - torque_res
            dtorque_err = dtorque_cmd[i] - dtorque_res
            torque_motor = torque_cmd[i] + Kt * torque_err + Ks * dtorque_err

            #data update
            time_data.append(time)
            qm_data.append(springy_rotor.xvec[0])
            dqm_data.append(springy_rotor.xvec[1])
            ql_data.append(springy_rotor.xvec[2])
            dql_data.append(springy_rotor.xvec[3])
            torque_cmd_data.append(torque_cmd[i])
            torque_data.append(torque_res)
            dtorque_data.append(dtorque_res)
            torque_err_data.append(torque_err)
            dtorque_err_data.append(dtorque_err)
            """ controller end """

        """ plant """

        # derivative calculation
        springy_rotor.dxvec = springy_rotor.calc_deri(torque_motor, 0.0, torque_reac[i])
        # euler-integration
        springy_rotor.update()

        """ plant end """

    # data plot
    from matplotlib import pyplot as plt
    plt.figure(figsize=(10, 7))
    plt.subplot(121)
    plt.plot(time_data, torque_cmd_data, label="torque cmd")
    plt.plot(time_data, torque_data, label="torque res")
    plt.legend()
    plt.grid()
    plt.ylabel('theta [rad]')
    plt.xlabel('time [s]')

    plt.subplot(122)
    plt.plot(time_data, torque_err_data, label="torque error")
    plt.legend()
    plt.grid()
    plt.xlabel('time [s]')

    plt.show()


if __name__ == "__main__":

    simulation_time = 3.0
    sampling_time = 0.0001 # 100 us

    #################################################################
    ### example 4 two-inertia torque control
    #################################################################
    Jm=0.005    # motor inertia [kgm^2]
    Bm=0.01    # motor viscous friction [Nm/rad^s]
    Jl=0.1    # link inertia [kgm^2]
    Bl=0.01    # link viscous friction [Nm/s]
    Ks=1000.0  # coupling spring [Nm/rad]
    Ds=0.0    # coupling dampling [Nm/rads]
    plant_parameters = [sampling_time, Jm, Bm, Jl, Bl, Ks, Ds]
    fr = np.sqrt((Ks/Jl) * (1 + Jl / Jm)) / (2.0*np.pi)
    far = np.sqrt(Ks/Jl) / (2.0*np.pi)
    print('Resonant frequency {0:6.2f}'.format(fr))
    print('Anti-resonant frequency {0:6.2f}'.format(far))

    # Command
    torque_cmd = []
    dtorque_cmd = []
    for i in range(int(simulation_time/sampling_time)):
        time = i * sampling_time
        if time <= 0.5:
            torque_cmd_tmp = 0.0
            dtorque_cmd_tmp = 0.0
        else:
            torque_cmd_tmp = 1.0
            dtorque_cmd_tmp = 0.0
        torque_cmd.append(torque_cmd_tmp)
        dtorque_cmd.append(dtorque_cmd_tmp)

    # Disturbance
    torque_reac = []
    for i in range(int(simulation_time/sampling_time)):
        time = i * sampling_time
        torque_reac_tmp = 0.0
        torque_reac.append(torque_reac_tmp)

    external_inputs = [torque_cmd, dtorque_cmd, torque_reac]

    # Position gains
    Kt = 100.0                         # P gain for velocity control loop
    Ks = 1.0                           # P gain for position control loop
    wt = 10.0                          # I gain for velocity control loop
    control_parameters = [Kt, Ks, wt]

    # Simulation
    two_inertia_torque_control(simulation_time, plant_parameters, control_parameters, external_inputs)
