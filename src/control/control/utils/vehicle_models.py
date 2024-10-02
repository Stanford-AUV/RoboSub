from scipy.integrate import solve_ivp
import numpy as np


def vehicle_models(x_robot, u_robot, model_type, Ts):
    if model_type == "Type I":
        sol = solve_ivp(vehicle_model_2D_Type1, [0, Ts], x_robot, args=(u_robot,))
        x_robot_next = sol.y[:, -1]
    elif model_type == "Type II":
        sol = solve_ivp(vehicle_model_2D_Type2, [0, Ts], x_robot[:2], args=(u_robot,))
        x_robot_next = np.concatenate([sol.y[:, -1], [u_robot[1]]])
    else:
        print("Model type is not compatible")
        x_robot_next = np.zeros(len(x_robot))

    return x_robot_next


def vehicle_model_2D_Type1(t, x, u):
    # Model input includes vehicle speed and heading rate
    psi = x[2]
    ur = u[0]
    r = u[1]
    dx = [ur * np.cos(psi), ur * np.sin(psi), r]
    return dx


def vehicle_model_2D_Type2(t, x, u):
    # Model input includes vehicle speed and heading
    psi = u[1]
    ur = u[0]
    dx = [ur * np.cos(psi), ur * np.sin(psi)]
    return dx
