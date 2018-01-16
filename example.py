import numpy as np
from simulation import *

def IPMSP_positioning():

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
    rigid_rotor = RigidRotor(J = 0.1, D = 0.001, sampling_time = sim_env.sampling_time, control_sampling_time=0.001)

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
            dtheta_cmd = 20.0 * (theta_cmd - rigid_rotor.xvec[0])

            # velocity controller (P velocity feedback control)
            #dtheta_cmd = 50.0
            iq_cmd = 3.0 * (dtheta_cmd - rigid_rotor.xvec[1])

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
            angle_data.append(rigid_rotor.xvec[0])
            omega_data.append(rigid_rotor.xvec[1])
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
        rigid_rotor.dxvec = rigid_rotor.calc_deri(IPMSM_motor.torque, torque_reac)
        IPMSM_motor.dxvec = IPMSM_motor.calc_deri(vd, vq, rigid_rotor.xvec[1])
        # euler-integration
        rigid_rotor.update()
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

if __name__ == "__main__":
    IPMSP_positioning()

