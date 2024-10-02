import sympy as sp


def path_eq(pathtype):
    t = sp.symbols("t")

    if pathtype == "line":
        vd = 0.02
        a = 30
        x0, y0 = 0, 0
        pd = sp.Matrix([x0 + a * t + 1e-10 * sp.cos(t), y0])

    elif pathtype == "circle":
        vd = 0.05
        a = 10
        x0, y0 = 0, 0
        pd = sp.Matrix([x0 + a * sp.cos(t), y0 + a * sp.sin(t)])

    elif pathtype == "sin":
        vd = 0.2
        a = 10
        omega = 0.05
        phi = 0
        d = 10
        pd = sp.Matrix([a * sp.sin(omega * t + phi) + d, t])

    elif pathtype == "polynomial":
        vd = 0.005
        a = sp.Matrix([0, 0, -1.797, 9.905, 10.891, 1])
        b = sp.Matrix([0, 0, -23.539, 37.078, -3.539, 1])
        x_offset = 20
        y_offset = -10
        z = t
        phi = sp.Matrix([z**5, z**4, z**3, z**2, z, 1])
        pd = sp.Matrix([a.dot(phi) + x_offset, b.dot(phi) + y_offset])

    elif pathtype == "Bernoulli":
        vd = 0.02
        a = 20
        z = 1 + sp.sin(t) ** 2
        pd = sp.Matrix([a * sp.cos(t) / z, a * sp.sin(t) * sp.cos(t) / z])

    elif pathtype == "Heart":
        vd = 0.02
        x_offset = 20
        y_offset = -20
        pd = sp.Matrix(
            [
                16 * sp.sin(t) ** 3 + x_offset,
                13 * sp.cos(t)
                - 5 * sp.cos(2 * t)
                - 2 * sp.cos(3 * t)
                - sp.cos(4 * t)
                + y_offset,
            ]
        )

    elif pathtype == "Complicated":
        vd = 0.01
        a = 5
        b = 2
        x_offset = 15
        y_offset = -15
        pd = sp.Matrix(
            [
                a * sp.sin(t) + b * sp.sin(2 * t) + x_offset,
                a * sp.cos(t) + b * sp.cos(2 * t) + y_offset,
            ]
        )

    elif pathtype == "Complicated2":
        vd = 0.01
        a = 10
        b = 5
        x_offset = 10
        y_offset = -10
        # Using a combination of trigonometric functions and absolute values to create sharp edges
        pd = sp.Matrix(
            [
                a * sp.sin(t) * sp.cos(t**2) + b * t**2 + x_offset,
                a * sp.sin(t**2) - b * sp.cos(2 * t) + t + y_offset,
            ]
        )

    # Compute derivatives
    d_pd = pd.diff(t)
    dd_pd = d_pd.diff(t)

    # Convert to callable functions
    pd_func = sp.lambdify(t, pd, "numpy")
    d_pd_func = sp.lambdify(t, d_pd, "numpy")
    dd_pd_func = sp.lambdify(t, dd_pd, "numpy")

    return pd_func, d_pd_func, dd_pd_func, vd
