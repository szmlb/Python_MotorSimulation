import numpy as np
from simulation import *

def SPMSM_positioning(simulation_time, plant_parameters, control_parameters, external_inputs):

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
    L=plant_parameters[4]
    Ke=plant_parameters[5]
    P=plant_parameters[6]

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
    SPMSM_motor = SPMSM(L=L, Ke=Ke, R=R, P=P, sampling_time=sim_env.sampling_time, control_sampling_time=sampling_time)
    rigid_rotor = RigidRotor(J=J, B=B, sampling_time = sim_env.sampling_time, control_sampling_time=sampling_time)

    # main loop 10[sec]
    for i in range(int(simulation_time*(1/sim_env.sampling_time))):
        time = i * sim_env.sampling_time

        control_delay = (int)(SPMSM_motor.control_sampling_time/sim_env.sampling_time) #[sample]
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
            id_err = id_cmd - SPMSM_motor.xvec[0]
            id_err_int = id_err_int + id_err * SPMSM_motor.control_sampling_time
            vd = Kpi * id_err + Kii * id_err_int
            # q current control
            iq_err = iq_ref - SPMSM_motor.xvec[1]
            iq_err_int = iq_err_int + iq_err * SPMSM_motor.control_sampling_time
            vq = Kpi * iq_err + Kii * iq_err_int

            #data update
            time_data.append(time)
            id_data.append(SPMSM_motor.xvec[0])
            iq_data.append(SPMSM_motor.xvec[1])
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
        rigid_rotor.dxvec = rigid_rotor.calc_deri(SPMSM_motor.torque, torque_reac[i])
        SPMSM_motor.dxvec = SPMSM_motor.calc_deri(vd, vq, rigid_rotor.xvec[1])
        # euler-integration
        rigid_rotor.update()
        SPMSM_motor.update()

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
    ### example 1 SPMSM positioning
    #################################################################
    L=3.9*0.001
    Ke=47.21*0.001
    R=154.9*0.001
    P=3
    J=0.1
    B=0.001
    plant_parameters = [sampling_time, J, B, R, L, Ke, P]

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
    SPMSM_positioning(simulation_time, plant_parameters, control_parameters, external_inputs)

    """
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
    """

    """
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
    """

    """
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
    """
