import matplotlib.pyplot as plt


def animation_1vehicles(pd, p):
    """
    Plots the desired path and the actual path followed by the robot.

    Parameters:
    pd (np.ndarray): The desired path (2D array of path points).
    p (np.ndarray): The actual path followed by the robot (2D array of robot positions).
    """
    plt.figure(figsize=(10, 6))

    # Plot the desired path
    plt.plot(pd[:, 0], pd[:, 1], "r-", label="Desired Path", linewidth=4)

    # Plot the actual path followed by the robot
    plt.plot(p[0, :], p[1, :], "b-", label="Actual Path", linewidth=2)

    # Labels and legend
    plt.title("Desired Path vs Actual Path of the Robot")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.grid(True)

    # Display the plot
    plt.show()
