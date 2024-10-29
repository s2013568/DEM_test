import numpy as np
import logging
import time
from utils import *
from model import *

# Assuming init, euler, and Helbing_Model_2D functions are defined elsewhere in your script

def simulation(N_ped, dt, t_end, state, once, f, fps):
    param = (0.5, 1.34, 10, 0, True, 2.1, 0.3, 2)
    t = 0
    frame = 0
    iframe = 0
    ids = np.arange(N_ped)
    np.savetxt(f, [], header="id\t time\t x\t y\t vx\t vy")
    last_vx, last_vy = 0, 0  # Variables to store the last mean velocities
    
    while t <= t_end:
        if frame % int(1 / (dt * fps)) == 0:
            last_vx = np.mean(state[2, :])  # Update with the latest mean x-velocity
            last_vy = np.mean(state[3, :])  # Update with the latest mean y-velocity
            
            T = t * np.ones(N_ped)
            output = np.vstack([ids, T, state[0, :], state[1, :], state[2, :], state[3, :]]).transpose()
            np.savetxt(f, output, fmt="%d\t%7.3f\t%7.3f\t%7.3f\t%7.3f\t%7.3f")
            f.flush()
            iframe += 1

        t, state, flag = euler(t, dt, state, Helbing_Model_2D, param)

        frame += 1
    
    return last_vx, last_vy  # Return the last mean velocities

def run_simulations():
    logfile = "log.txt"
    open(logfile, "w").close()  # Initialize log file
    logging.basicConfig(
        filename=logfile,
        level=logging.DEBUG,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )
    param = (0.5, 1.34, 10, 0, True, 2.1, 0.3, 2)
    tau, v0, Length, Width, periodic, A, B, delta_t = param
    dt = 0.01
    t_end = 1.5
    fps = 8

    velocities = []  # List to store final mean velocities for each N_ped

    with open("mean_velocities.txt", "w") as outfile:
        outfile.write("N_ped\tMean vx\tMean vy\n")  # Write headers to the file

        for N_ped in [4]:
            prefix = "tau%.2f_v0%.2f_Length%.2f_Width%.2f_periodic%s_A%.2f_B%.2f_delta_t%.2f" % (tau, v0, Length, Width, str(periodic), A, B, delta_t)
            filename = f"traj_{prefix}_N{N_ped}.txt"
            
            with open(filename, "wb") as f:
                state = init(N_ped, 10, 0)
                logging.info(
                    f"Starting simulation with N_ped={N_ped}, tau={tau}, v0={v0}, Length={Length}, Width={Width}, periodic={periodic}, A={A}, B={B}, delta_t={delta_t}"
                )
                
                t1 = time.perf_counter()
                last_vx, last_vy = simulation(N_ped, dt, t_end, state, 1, f, fps)
                t2 = time.perf_counter()
                
                logging.info(
                    "Simulation time: %.3f s (%.2f min) with tau=%.2f and N_ped=%d"
                    % ((t2 - t1), (t2 - t1) / 60, tau, N_ped)
                )
                
                velocities.append((N_ped, last_vx, last_vy))
                outfile.write(f"{N_ped}\t{last_vx:.4f}\t{last_vy:.4f}\n")  # Write results to file

if __name__ == "__main__":
    run_simulations()