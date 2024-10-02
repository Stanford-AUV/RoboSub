import numpy as np
from scipy.optimize import fsolve

psiP_old = None
psiP_out_old = None


def path_state(pd, d_pd, dd_pd, controller, gamma, p):
    global psiP_old, psiP_out_old

    if controller in ["Method 1", "Method 3"]:
        # Find the point on the path closest to the vehicle
        f = lambda t: np.dot(pd(t).reshape(-1) - p.reshape(-1), d_pd(t).reshape(-1))
        gamma_opt = fsolve(f, gamma)[0]
    else:
        gamma_opt = gamma  # Use the last gamma

    # Compute the first and second derivative of pd with the optimal gamma
    t = gamma_opt
    pd_val = pd(t).flatten()
    pd_gamma = d_pd(t).flatten()
    b = dd_pd(t).flatten()
    kappa = (pd_gamma[0] * b[1] - pd_gamma[1] * b[0]) / np.linalg.norm(
        pd_gamma
    ) ** 3  # Curvature of the path

    # Open the space of the angle to R
    psiP_new = np.arctan2(pd_gamma[1], pd_gamma[0])
    if psiP_old is None:
        psiP_old = psiP_new
        psiP_out_old = psiP_new
        psiP_out = psiP_new
    else:
        psiP_out = alg_convert(psiP_new, psiP_old, psiP_out_old)
        psiP_old = psiP_new
        psiP_out_old = psiP_out

    # Convert scalar values to arrays to match the dimensions for concatenation
    kappa = np.array([kappa])
    psiP_out = np.array([psiP_out])
    gamma_opt = np.array([gamma_opt])

    # Return the path state
    x_path = np.concatenate([pd_val, pd_gamma, kappa, psiP_out, gamma_opt])

    return x_path


def alg_convert(alg_new, alg_old, alg_out_old):
    alg_e = alg_new - alg_old
    if alg_e > 3 * np.pi / 2:
        alg_out = alg_out_old - 2 * np.pi + alg_e
    elif alg_e < -3 * np.pi / 2:
        alg_out = alg_out_old + 2 * np.pi + alg_e
    else:
        alg_out = alg_out_old + alg_e
    return alg_out
