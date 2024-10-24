#!/usr/bin/python
# -------------------------------------------------------------
# from numpy import *
# import os
import logging
import time
from utils import *

# ---------------------- Parameter ---------------------------------------
fps = 8  # frames per second
dt = 0.001  # [s] integrator step length
t_end = 1000  # [s] integration time
N_ped = 49  # number of pedestrians delta YN= 1.5
Length = 200  # [m] length of corridor. *Closed boundary conditions*
# ========================= SOLVER
RK4 = 0  # 1 --> RK4.
EULER = 1  # if RK4==0 and EULER==0 ---> Heun
HEUN = 0
once = 1
# ------------------------- logging ----------------------------------------
rho = float(N_ped) / Length


# ======================================================
def get_state_vars(state):
    """
    state variables and dist
    """
    x_n = state[0, :]  # x_n
    x_m = np.roll(x_n, -1)  # x_{n+1}
    x_m[-1] += Length  # the first one goes ahead by Length
    dx_n = state[1, :]  # dx_n
    dx_m = np.roll(dx_n, -1)  # dx_{n+1}
    dist = x_m - x_n
    if (dist <= 0).any():
        logging.critical("CRITICAL: Negative distances")
    return x_n, x_m, dx_n, dx_m, dist


# ======================================================
def model(t, state):
    """
    log model
    """
    x_n, x_m, dx_n, dx_m, dist = get_state_vars(state)
    if (x_n != np.sort(x_n)).any():  # check if pedestrians are swaping positions
        swapping = x_n != np.sort(x_n)  # swapping is True if there is some swapping
        swaped_dist = x_n[swapping]
        swaped_ped = [i for i, v in enumerate(swapping) if v == True]

        print("swaped_peds".format(swaped_ped))
        print("distances ".format(swaped_dist))
        return state, 0

    f_drv = (v0 - dx_n) / tau
    c = np.e - 1
    Dxn = dist
    Dmin = 2 * a0 + av * (dx_n + dx_m)
    R = 1 - Dxn / Dmin
    eps = 0.01
    R = eps * np.log(1 + np.exp(R / eps))
    f_rep = -v0 / tau * np.log(c * R + 1)
    ########################
    a_n = f_rep + f_drv
    #######################
    x_n_new = dx_n
    dx_n_new = a_n
    new_state = np.vstack([x_n_new, dx_n_new])
    return new_state, 1


# ======================================================
def simulation(N_ped, dt, t_end, state, once, f):
    t = 0
    frame = 0
    iframe = 0
    ids = np.arange(N_ped)
    np.savetxt(f, [], header="id\t time\t x\t v")
    while t <= t_end:
        if frame % (1 / (dt * fps)) == 0:
            logging.info(
                "time=%6.1f (<= %4d) frame=%3.3E v=%f  vmin=%f  std=+-%f"
                % (
                    t,
                    t_end,
                    frame,
                    np.mean(state[1, :]),
                    min(state[1, :]),
                    np.std(state[1, :]),
                )
            )
            T = t * np.ones(N_ped)
            output = np.vstack([ids, T, state]).transpose()
            np.savetxt(f, output, fmt="%d\t %7.3f\t %7.3f\t %7.3f")
            f.flush()

            iframe += 1

        if RK4:  # Runge-Kutta
            t, state, flag = rk4(t, dt, state, model)
        elif EULER:  # Euler
            t, state, flag = euler(t, dt, state, model)
        elif HEUN:  # Heun
            t, state, flag = heun(t, dt, state, model)

        if not flag:
            if once:
                t = t_end - dt
                print("t_end %f" % t_end)
                if RK4:
                    logging.info("Solver:\t RK4")
                elif EULER:  # Euler
                    logging.info("Solver:\t EULER")
                else:
                    logging.info("Solver:\t HEUN")
                once = 0

        frame += 1


# ======================================================
# set up figure and animation

if __name__ == "__main__":
    logfile = "log.txt"
    open(logfile, "w").close()  # touch the file
    logging.basicConfig(
        filename=logfile,
        level=logging.DEBUG,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )

    Dyn = float(Length) / N_ped
    # ============================
    av = 0.0
    v0 =  1.0
    a0 = 1.0
    tau = 1.0
    # tau_values = np.arange(0.5, 1.6, 0.1)  # Generate tau values from 0.5 to 1.5 in steps of 0.1
rho_values = np.arange(0.5, 4.6, 0.5)  # Densities from 0.5 to 4 in steps of 0.5
# tau_values = [0.5, 0.7, 0.9, 1.0, 1.2, 1.4]  # Example tau values you want to test

# avs = [6.0]

# v0s= np.arange(0.2, 1.2, 0.2)

# Loop over each tau value
# for av in avs:
    # Loop over each density
for rho in rho_values:
    N_ped = int(rho * Length)  # Calculate N_ped based on the current density
    
    # Prefix for file name, includes tau and density (rho)
    prefix = "%d_av%.2f_v0%.2f_tau%.2f_rho%.2f" % (N_ped, av, v0, tau, rho)
    filename = "traj_" + prefix + ".txt"
    f = open(filename, "wb")
    
    logging.info("start initialization with %d peds and density %.2f" % (N_ped, rho))
    state = init(N_ped, Length)
    
    logging.info(
        "simulation with v0=%.2f, av=%.2f, dt=%.4f, rho=%.2f, tau=%.2f"
        % (v0, av, dt, rho, tau)
    )
    
    print("filename %s" % filename)
    t1 = time.perf_counter()
    ######################################################
    simulation(N_ped, dt, t_end, state, once, f)
    ######################################################
    t2 = time.perf_counter()
    logging.info(
        "simulation time %.3f [s] (%.2f [min]) with tau=%.2f and density=%.2f"
        % ((t2 - t1), (t2 - t1) / 60, tau, rho)
    )
    
    logging.info("close %s" % filename)
    f.close()