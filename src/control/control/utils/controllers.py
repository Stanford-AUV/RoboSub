import numpy as np
import time
import casadi as ca


def sat(u, l_bound, u_bound):
    return np.maximum(np.minimum(u, u_bound), l_bound)


def PFcontrollers(x_robot, x_path, upf, controller, vd, Ts, MPC_PF, l_bound, u_bound):
    p = x_robot[:2]
    psi = x_robot[2]

    pd = x_path[:2]
    pd_gamma = x_path[2:4]
    hg = np.sqrt(pd_gamma[0] ** 2 + pd_gamma[1] ** 2)
    v_robot, r_robot, v_gamma = upf
    psiP = x_path[5]
    kappa = x_path[4]
    gamma = x_path[-1]

    # Compute PF error
    if controller == "Method 1":
        start_time = time.time()
        upf = Method1(p, psi, pd, psiP, kappa, v_robot, hg, vd)
        exc_time = time.time() - start_time
        upf[1] = sat(upf[1], l_bound[1], u_bound[1])

    elif controller == "Method 2":
        start_time = time.time()
        upf = Method2(p, psi, pd, psiP, kappa, v_gamma, v_robot, hg, vd)
        exc_time = time.time() - start_time
        upf[1:3] = sat(upf[1:3], l_bound[1:3], u_bound[1:3])

    elif controller == "Method 3":
        start_time = time.time()
        upf = Method3(p, psi, pd, psiP, kappa, v_robot, hg, vd)
        exc_time = time.time() - start_time

    elif controller == "Method 4":
        start_time = time.time()
        upf = Method4(p, psi, pd, psiP, kappa, v_robot, hg, vd)
        exc_time = time.time() - start_time
        upf[2] = sat(upf[2], l_bound[2], u_bound[2])

    elif controller == "Method 5":
        start_time = time.time()
        upf = Method5(p, psi, pd, psiP, kappa, v_robot, hg, vd, gamma, MPC_PF)
        exc_time = time.time() - start_time

    elif controller == "Method 6":
        start_time = time.time()
        upf = Method6(p, psi, pd, pd_gamma, psiP, kappa, v_robot, hg, vd, Ts)
        exc_time = time.time() - start_time
        upf = sat(upf, l_bound, u_bound)

    elif controller == "Method 7":
        start_time = time.time()
        upf = Method7(p, psi, pd, pd_gamma, psiP, vd, Ts, gamma, MPC_PF)
        exc_time = time.time() - start_time

    return upf, exc_time


# Method 1 implementation (similar methods for Method2 to Method7 can be defined)
def Method1(p, psi, pd, psiP, kappa, v_robot, hg, vd):
    RI_F = np.array([[np.cos(psiP), np.sin(psiP)], [-np.sin(psiP), np.cos(psiP)]])
    e_P = np.dot(RI_F, (p - pd))
    y1 = e_P[1]
    psie = psi - psiP

    k1, k2, k3, k_delta, theta, Delta_h = ConPara1()

    delta = -theta * np.tanh(k_delta * y1)
    ydot = v_robot * np.sin(psie)
    delta_dot = -theta * k_delta * (1 - np.tanh(k_delta * y1) ** 2) * ydot
    psi_tilde = psie - delta

    u_d = hg * vd
    u = u_d
    uP = u * np.cos(psie) / (1 - kappa * y1)
    v_gamma = uP / hg

    if psie == delta:
        r = delta_dot - kappa * v_gamma * hg
    else:
        r = (
            kappa * uP
            + delta_dot
            - k1 * psi_tilde
            - k2 * y1 * u * (np.sin(psie) - np.sin(delta)) / psi_tilde
        )

    return np.array([u, r, v_gamma])


def Method2(p, psi, pd, psiP, kappa, v_gamma, v_robot, hg, vd):
    RI_F = np.array([[np.cos(psiP), np.sin(psiP)], [-np.sin(psiP), np.cos(psiP)]])
    e_P = np.dot(RI_F, (p - pd))
    s1 = e_P[0]
    y1 = e_P[1]
    psie = psi - psiP

    k1, k2, k3, k_delta, theta, Delta_h = ConPara1()

    delta = -theta * np.tanh(k_delta * y1)
    ydot = v_robot * np.sin(psie) - hg * kappa * v_gamma * s1
    delta_dot = -theta * k_delta * (1 - np.tanh(k_delta * y1) ** 2) * ydot
    psi_tilde = psie - delta

    u_d = hg * vd
    u = u_d
    uP = u * np.cos(psie) + k3 * s1
    v_gamma = uP / hg

    if psie == delta:
        r = delta_dot - kappa * v_gamma * hg
    else:
        r = (
            kappa * uP
            + delta_dot
            - k1 * psi_tilde
            - k2 * y1 * u * (np.sin(psie) - np.sin(delta)) / psi_tilde
        )

    return np.array([u, r, v_gamma])


def Method3(p, psi, pd, psiP, kappa, v_robot, hg, vd):
    RI_F = np.array([[np.cos(psiP), np.sin(psiP)], [-np.sin(psiP), np.cos(psiP)]])
    e_P = np.dot(RI_F, (p - pd))
    s1 = e_P[0]
    y1 = e_P[1]

    k1, k2, k3, k_delta, theta, Delta_h = ConPara1()

    # Controller
    u_d = hg * vd
    u = u_d
    v_gamma = 0
    psi_los = psiP + np.arctan(-y1 / Delta_h)

    return np.array([u, psi_los, v_gamma])


def Method4(p, psi, pd, psiP, kappa, v_robot, hg, vd):
    RI_F = np.array([[np.cos(psiP), np.sin(psiP)], [-np.sin(psiP), np.cos(psiP)]])
    e_P = np.dot(RI_F, (p - pd))
    s1 = e_P[0]
    y1 = e_P[1]
    psie = psi - psiP

    k1, k2, k3, k_delta, theta, Delta_h = ConPara1()

    # Controller
    u_d = hg * vd
    u = u_d
    uP = u * np.cos(psie) + k3 * s1
    v_gamma = uP / hg
    psi_los = psiP + np.arctan(-y1 / Delta_h)

    return np.array([u, psi_los, v_gamma])


def Method5(p, psi, pd, psiP, kappa, v_robot, hg, vd, gamma, MPC_PF):
    # Persistent variables (only initialized once)
    if not hasattr(Method5, "w0"):
        Method5.w0 = (
            ca.vertcat(*MPC_PF["w0"])
            if isinstance(MPC_PF["w0"], list)
            else MPC_PF["w0"]
        )
        Method5.lbw = (
            ca.vertcat(*MPC_PF["lbw"])
            if isinstance(MPC_PF["lbw"], list)
            else MPC_PF["lbw"]
        )
        Method5.ubw = (
            ca.vertcat(*MPC_PF["ubw"])
            if isinstance(MPC_PF["ubw"], list)
            else MPC_PF["ubw"]
        )
        Method5.lbg = (
            ca.vertcat(*MPC_PF["lbg"])
            if isinstance(MPC_PF["lbg"], list)
            else MPC_PF["lbg"]
        )
        Method5.ubg = (
            ca.vertcat(*MPC_PF["ubg"])
            if isinstance(MPC_PF["ubg"], list)
            else MPC_PF["ubg"]
        )

    RI_F = np.array([[np.cos(psiP), np.sin(psiP)], [-np.sin(psiP), np.cos(psiP)]])
    e_P = np.dot(RI_F, (p - pd))
    s1 = e_P[0]
    y1 = e_P[1]
    psie = psi - psiP

    # Calculate control input u
    u_d = hg * vd
    u = float(u_d)  # Ensure u is a scalar

    # Prepare the initial state vector for the optimization problem
    x0 = ca.DM([s1, y1, psie, gamma])  # Convert to CasADi DM type
    nx = x0.size1()

    # Set initial conditions for the optimization solver
    Method5.w0 = ca.vertcat(x0, Method5.w0[nx:])  # Only replace the first nx elements
    Method5.lbw = ca.vertcat(x0, Method5.lbw[nx:])
    Method5.ubw = ca.vertcat(x0, Method5.ubw[nx:])

    # Solve the optimization problem using the provided MPC solver
    sol = MPC_PF["nlp"](
        x0=Method5.w0,
        lbx=Method5.lbw,
        ubx=Method5.ubw,
        lbg=Method5.lbg,
        ubg=Method5.ubg,
    )

    w_opt = sol["x"]
    u_mpc = w_opt[4:6]  # Extract the control inputs (r_d and v_gamma) from the solution

    r_d = float(u_mpc[0])  # Ensure r_d is a scalar
    v_gamma = float(u_mpc[1])  # Ensure v_gamma is a scalar

    # Store the solution for future iterations
    Method5.w0 = w_opt

    # Return the control inputs as a NumPy array
    return np.array([u, r_d, v_gamma])


def Method6(p, psi, pd, pd_gamma, psiP, kappa, v_robot, hg, vd, Ts):
    # Persistent variable to retain state between calls
    global gamma_dot_old
    if "gamma_dot_old" not in globals():
        gamma_dot_old = 0

    delta, Delta_inv, epsilon, Kk, kz = ConPara2()

    # Rotation matrix from body frame to inertial frame
    RB_I = np.array([[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]])

    # Positional error in the body frame
    e_pos = np.dot(RB_I.T, (p - pd)) - epsilon

    # Gamma error (difference between current and desired velocity)
    e_gamma = gamma_dot_old - vd

    # PF control law (velocity and gamma_dot control)
    ud = np.dot(
        Delta_inv, (-np.tanh(np.dot(Kk, e_pos)) + np.dot(RB_I.T, pd_gamma) * vd)
    )

    # Compute second derivative of gamma
    gamma_ddot = -kz * e_gamma + np.dot(e_pos.T, np.dot(RB_I.T, pd_gamma))

    # Saturate gamma_ddot within limits
    gamma_ddot = sat(gamma_ddot, -0.005, 0.005)

    # Update gamma_dot with a simple integrator
    gamma_dot = gamma_dot_old + Ts * gamma_ddot
    gamma_dot = sat(gamma_dot, -0.005, 0.2)

    # Store the updated gamma_dot for the next iteration
    gamma_dot_old = gamma_dot

    return np.concatenate((ud, [gamma_dot]))


def Method7(p, psi, pd, pd_gamma, psiP, vd, Ts, gamma, MPC_PF):
    # Persistent variables initialization (if not already done)
    if not hasattr(Method7, "w0"):
        Method7.w0 = ca.DM(MPC_PF["w0"])  # Ensure CasADi DM type
        Method7.lbw = ca.DM(MPC_PF["lbw"])  # Ensure bounds are CasADi DM type
        Method7.ubw = ca.DM(MPC_PF["ubw"])
        Method7.lbg = ca.DM(MPC_PF["lbg"])
        Method7.ubg = ca.DM(MPC_PF["ubg"])

    # Retrieve persistent variables
    persistent_w = Method7.w0
    persistent_lbw = Method7.lbw
    persistent_ubw = Method7.ubw
    persistent_lbg = Method7.lbg
    persistent_ubg = Method7.ubg

    delta, Delta_inv, epsilon, Kk, kz = ConPara2()

    # Compute error in body frame
    RB_I = np.array([[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]])
    eB = np.dot(RB_I.T, (p - pd)) - epsilon

    # Set initial state vector for the optimization problem
    x0 = ca.DM(np.hstack((eB, psi, gamma)))  # Convert initial state to CasADi DM type
    nx = x0.size1()

    # Update the persistent variables with the initial state
    persistent_w = ca.vertcat(
        x0, persistent_w[nx:]
    )  # Update only the first nx elements
    persistent_lbw = ca.vertcat(x0, persistent_lbw[nx:])
    persistent_ubw = ca.vertcat(x0, persistent_ubw[nx:])

    # Solve the nonlinear optimization problem
    sol = MPC_PF["nlp"](
        x0=persistent_w,
        lbx=persistent_lbw,
        ubx=persistent_ubw,
        lbg=persistent_lbg,
        ubg=persistent_ubg,
    )

    w_opt = sol["x"]
    upf = np.array(
        w_opt[4:7]
    ).flatten()  # Extract control inputs (u, r, v_gamma) and flatten

    # Update the persistent variables with the new solution for future calls
    Method7.w0 = w_opt

    # Return the control inputs as a flat array
    return upf


def ConPara1():
    k1 = 1
    k2 = 1
    k3 = 0.1
    theta = 0.1 * np.pi / 4
    k_delta = 2
    Delta_h = 5
    return k1, k2, k3, k_delta, theta, Delta_h


def ConPara2():
    # Control parameters for Method 6 and Method 7
    delta = -0.2
    Delta = np.array([[1, 0], [0, -delta]])
    Delta_inv = np.linalg.inv(Delta)
    epsilon = np.array([delta, 0])

    # Gain matrices for the controller
    kx = 0.1
    ky = 0.05
    Kk = np.array([[kx, 0], [0, ky]])
    kz = 1

    return delta, Delta_inv, epsilon, Kk, kz
