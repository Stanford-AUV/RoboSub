import casadi as ca


def MPC_setup(Ts, d_pd, dd_pd, l_bound, u_bound, vd, controller):
    N = 10  # Number of control intervals
    gamma = ca.SX.sym("gamma")

    d_pd_val = d_pd(gamma)
    dd_pd_val = dd_pd(gamma)
    hg = ca.norm_2(d_pd_val)
    kappa = (d_pd_val[0] * dd_pd_val[1] - d_pd_val[1] * dd_pd_val[0]) / hg**3

    if controller == "Method 5":
        return NLP_Method5(
            Ts, d_pd_val, dd_pd_val, hg, kappa, gamma, l_bound, u_bound, vd, N
        )
    elif controller == "Method 7":
        return NLP_Method7(
            Ts, d_pd_val, dd_pd_val, hg, kappa, gamma, l_bound, u_bound, vd, N
        )


def NLP_Method5(Ts, d_pd, dd_pd, hg, kappa, gamma, l_bound, u_bound, vd, N):
    T = Ts * N  # Time horizon

    # Define system state
    s1 = ca.SX.sym("s1")
    y1 = ca.SX.sym("y1")
    psie = ca.SX.sym("psie")
    x = ca.vertcat(s1, y1, psie, gamma)
    nx = x.size1()

    # Define inputs
    r = ca.SX.sym("r")
    v_g = ca.SX.sym("v_g")
    u = ca.vertcat(r, v_g)
    nu = u.size1()

    # Path following system
    xdot = ca.vertcat(
        vd * hg * ca.cos(psie) - v_g * hg * (1 - kappa * y1),
        vd * hg * ca.sin(psie) - kappa * hg * s1 * v_g,
        r - kappa * hg * v_g,
        v_g,
    )

    rmax, rmin = u_bound[1], l_bound[1]
    vmax, vmin = u_bound[2], l_bound[2]
    umax = ca.vertcat(rmax, vmax)
    umin = ca.vertcat(rmin, vmin)

    # Objective term
    Q = ca.diag([1, 5, 0.1])
    R = ca.diag([1, 1])
    ua = ca.vertcat(vd * ca.cos(psie) - v_g, r - kappa * hg * v_g)
    L = ca.mtimes(
        [ca.vertcat(s1, y1, psie).T, Q, ca.vertcat(s1, y1, psie)]
    ) + ca.mtimes([ua.T, R, ua])

    # Continuous time dynamics
    f = ca.Function("f", [x, u], [xdot, L])

    # Discrete time dynamics (using RK4)
    M = 4
    DT = T / N / M
    X0 = ca.MX.sym("X0", nx)
    U = ca.MX.sym("U", nu)
    X = X0
    Q = 0
    for j in range(M):
        k1, k1_q = f(X, U)
        k2, k2_q = f(X + DT / 2 * k1, U)
        k3, k3_q = f(X + DT / 2 * k2, U)
        k4, k4_q = f(X + DT * k3, U)
        X += DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        Q += DT / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)

    F = ca.Function("F", [X0, U], [X, Q], ["x0", "p"], ["xf", "qf"])

    # NLP setup
    w, w0, lbw, ubw = [], [], [], []
    J, g, lbg, ubg = 0, [], [], []

    # Initial condition
    Xk = X0
    xmax = ca.vertcat([ca.inf] * 4)
    xmin = ca.vertcat([-ca.inf] * 4)
    uzero = ca.vertcat([0] * 2)
    xzero = ca.vertcat([0] * 4)

    w.append(X0)
    lbw.append(xzero)
    ubw.append(xzero)
    w0.append(xzero)

    # Formulate NLP
    for k in range(N):
        Uk = ca.MX.sym(f"U_{k}", nu)
        w.append(Uk)
        lbw.append(umin)
        ubw.append(umax)
        w0.append(uzero)

        Fk = F(x0=Xk, p=Uk)
        Xk_end = Fk["xf"]
        J += Fk["qf"]

        Xk = ca.MX.sym(f"X_{k+1}", nx)
        w.append(Xk)
        lbw.append(xmin)
        ubw.append(xmax)
        w0.append(xzero)

        g.append(Xk_end - Xk)
        lbg.append(xzero)
        ubg.append(xzero)

    prob = {"f": J, "x": ca.vertcat(*w), "g": ca.vertcat(*g)}
    opts = {"ipopt.print_level": 0, "print_time": False}
    solver = ca.nlpsol("solver", "ipopt", prob, opts)

    return {"nlp": solver, "w0": w0, "lbw": lbw, "ubw": ubw, "lbg": lbg, "ubg": ubg}


def NLP_Method7(Ts, d_pd, dd_pd, hg, kappa, gamma, l_bound, u_bound, vd, N):
    T = Ts * N  # Time horizon

    # Define system state
    eB = ca.SX.sym("eB", 2)  # Path-following error
    psi = ca.SX.sym("psi")  # Vehicle orientation
    x = ca.vertcat(eB, psi, gamma)
    nx = x.size1()

    # Define inputs
    u_v = ca.SX.sym("u_v")
    r = ca.SX.sym("r")
    v_g = ca.SX.sym("v_g")
    u = ca.vertcat(u_v, r, v_g)
    nu = u.size1()

    # Path-following system dynamics
    S = ca.SX.zeros(2, 2)
    S[0, 1] = -r
    S[1, 0] = r
    delta = -0.2
    Delta = ca.SX([[1, 0], [0, -delta]])

    # Instead of using nested lists, construct RIB with vertcat and horzcat
    RIB = ca.vertcat(
        ca.horzcat(ca.cos(psi), ca.sin(psi)), ca.horzcat(-ca.sin(psi), ca.cos(psi))
    )

    xdot = ca.vertcat(S @ eB + Delta @ ca.vertcat(u_v, r) - RIB @ d_pd * v_g, r, v_g)

    umax = ca.vertcat(*u_bound)
    umin = ca.vertcat(*l_bound)

    # Objective term
    Q = ca.diag([0.5, 0.2])
    R = ca.diag([10, 10])
    ub = Delta @ ca.vertcat(u_v, r) - RIB @ d_pd * v_g
    L = ca.mtimes([eB.T, Q, eB]) + ca.mtimes([ub.T, R, ub]) + (v_g - vd) ** 2

    # Continuous time dynamics
    f = ca.Function("f", [x, u], [xdot, L])

    # Discrete time dynamics (using RK4)
    M = 4
    DT = T / N / M
    X0 = ca.MX.sym("X0", nx)
    U = ca.MX.sym("U", nu)
    X = X0
    Q = 0
    for j in range(M):
        k1, k1_q = f(X, U)
        k2, k2_q = f(X + DT / 2 * k1, U)
        k3, k3_q = f(X + DT / 2 * k2, U)
        k4, k4_q = f(X + DT * k3, U)
        X += DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        Q += DT / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)

    F = ca.Function("F", [X0, U], [X, Q], ["x0", "p"], ["xf", "qf"])

    # NLP setup
    w, w0, lbw, ubw = [], [], [], []
    J, g, lbg, ubg = 0, [], [], []

    # Initial condition
    xmax = ca.vertcat([ca.inf] * nx)
    xmin = ca.vertcat([-ca.inf] * nx)
    uzero = ca.vertcat([0] * nu)
    xzero = ca.vertcat([0] * nx)

    w.append(X0)
    lbw.append(xzero)
    ubw.append(xzero)
    w0.append(xzero)

    # Formulate NLP
    Xk = X0
    for k in range(N):
        Uk = ca.MX.sym(f"U_{k}", nu)
        w.append(Uk)
        lbw.append(umin)
        ubw.append(umax)
        w0.append(uzero)

        Fk = F(x0=Xk, p=Uk)
        Xk_end = Fk["xf"]
        J += Fk["qf"]

        Xk = ca.MX.sym(f"X_{k+1}", nx)
        w.append(Xk)
        lbw.append(xmin)
        ubw.append(xmax)
        w0.append(xzero)

        g.append(Xk_end - Xk)
        lbg.append(xzero)
        ubg.append(xzero)

    prob = {"f": J, "x": ca.vertcat(*w), "g": ca.vertcat(*g)}
    opts = {"ipopt.print_level": 0, "print_time": False}
    solver = ca.nlpsol("solver", "ipopt", prob, opts)

    return {
        "nlp": solver,
        "w0": ca.vertcat(*w0),
        "lbw": ca.vertcat(*lbw),
        "ubw": ca.vertcat(*ubw),
        "lbg": ca.vertcat(*lbg),
        "ubg": ca.vertcat(*ubg),
    }
