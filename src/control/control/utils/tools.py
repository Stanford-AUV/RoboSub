import numpy as np
import time

from control.utils.controllers import PFcontrollers
from control.utils.mpc import MPC_setup
from control.utils.path_eq import path_eq
from control.utils.path_state import path_state
from control.utils.vehicle_models import vehicle_models
from control.utils.animation import animation_1vehicles


def PFtools():
    # Initialization
    T = 500  # Simulation time (the time the robot will run)
    Ts = 0.2  # Sampling time (time step)
    N = int(T / Ts)
    t = 0
    p0 = np.array([25, -15])
    psi0 = np.pi / 2
    x_robot0 = np.array([p0[0], p0[1], psi0])

    gamma0 = 0.1
    path_type = "Complicated2"
    pd, d_pd, dd_pd, vd = path_eq(path_type)

    controller = "Method 4"
    umin, umax = 0, 1
    rmin, rmax = -0.2, 0.2
    vmin, vmax = -1, 1
    l_bound = np.array([umin, rmin, vmin])
    u_bound = np.array([umax, rmax, vmax])

    if controller in ["Method 5", "Method 7"]:
        MPC_PF = MPC_setup(Ts, d_pd, dd_pd, l_bound, u_bound, vd, controller)
    else:
        MPC_PF = None

    # Main Loop
    x_robot = np.zeros((3, N + 1))  # 3 states: x, y, psi, store for each time step
    x_robot[:, 0] = x_robot0
    x_path = []
    upf = np.array([0, 0, 0])
    gamma = [gamma0]
    compute_time = []
    path_compute_time = []

    for i in range(1, N + 1):
        print(i)

        t = np.append(t, i * Ts)

        # Step 1: Get the state of the path
        p = x_robot[:2, i - 1]  # Correct indexing for current position
        start_time = time.time()
        x_path.append(path_state(pd, d_pd, dd_pd, controller, gamma[-1], p))
        path_compute_time.append(time.time() - start_time)

        # Step 2: Compute path following controller
        upfi, exc_time = PFcontrollers(
            x_robot[:, i - 1],
            x_path[-1],
            upf,
            controller,
            vd,
            Ts,
            MPC_PF,
            l_bound,
            u_bound,
        )
        compute_time.append(exc_time + path_compute_time[-1])
        u_robot = upfi[:2]

        # Step 3: Update the state of the vehicle
        model_type = "Type II" if controller in ["Method 3", "Method 4"] else "Type I"

        x_robot[:, i] = vehicle_models(
            x_robot[:, i - 1], u_robot, model_type, Ts
        )  # Update the next state

        # Step 4: Update the path parameter
        u_gamma = upfi[2]
        if controller in ["Method 1", "Method 3"]:
            gamma.append(x_path[-1][-1])
        else:
            gamma.append(gamma[-1] + Ts * u_gamma)

    # Save Data
    x_path = np.array(x_path)
    upf = np.array(upf)
    # save_to_base(1)

    # Run animation
    pd = x_path[:, :2]
    p = x_robot[:2, :]

    animation_1vehicles(pd, p)


PFtools()
