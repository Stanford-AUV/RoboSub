import numpy as np

import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def oriented_bounding_box(points):
    """
    Computes an oriented bounding box for a set of points.

    Args:
        points (np.ndarray): A Nx3 array of 3D points.
    Returns:
        center (np.ndarray): The center of the bounding box.

        extents (np.ndarray): The half-lengths of the bounding box along each axis.

        rotation_matrix (np.ndarray): A 3x3 rotation matrix representing the box's orientation.
    """

    # Step 1: Compute the covariance matrix of the points
    covariance_matrix = np.cov(points, rowvar=False)

    # Step 2: Perform PCA (Eigen decomposition) on the covariance matrix
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
    # Sort eigenvalues and eigenvectors in descending order
    order = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    # Step 3: Rotate the points to align with the principal components
    rotated_points = points @ eigenvectors

    # Step 4: Find the min and max along each principal component
    min_bounds = rotated_points.min(axis=0)
    max_bounds = rotated_points.max(axis=0)

    # Step 5: Calculate the center and extents in the rotated frame
    center_rotated = (min_bounds + max_bounds) / 2
    extents = (max_bounds - min_bounds) / 2
    size = max_bounds - min_bounds  # width, height, depth

    # Step 6: Transform the center back to the original coordinate system
    center = eigenvectors @ center_rotated

    return center, size, eigenvectors


def plot_oriented_bounding_box(points, center, size, rotation_matrix):
    """
    Plots an oriented bounding box (OBB) along with a set of points in 3D space.

    Args:
        points (np.ndarray): A Nx3 array of 3D points.
        center (np.ndarray): The center of the bounding box.
        size (np.ndarray): The dimensions of the bounding box (width, height, depth).
        rotation_matrix (np.ndarray): A 3x3 rotation matrix representing the box's orientation.
    """
    # Define the 8 corners of the OBB in the local frame
    half_size = size / 2
    corners = np.array(
        [
            [-half_size[0], -half_size[1], -half_size[2]],
            [half_size[0], -half_size[1], -half_size[2]],
            [half_size[0], half_size[1], -half_size[2]],
            [-half_size[0], half_size[1], -half_size[2]],
            [-half_size[0], -half_size[1], half_size[2]],
            [half_size[0], -half_size[1], half_size[2]],
            [half_size[0], half_size[1], half_size[2]],
            [-half_size[0], half_size[1], half_size[2]],
        ]
    )

    # Transform corners to the global coordinate system
    transformed_corners = (rotation_matrix @ corners.T).T + center

    # Define the edges of the box for plotting
    edges = [
        [transformed_corners[i] for i in [0, 1, 2, 3]],  # Bottom face
        [transformed_corners[i] for i in [4, 5, 6, 7]],  # Top face
        [transformed_corners[i] for i in [0, 1, 5, 4]],  # Side edges
        [transformed_corners[i] for i in [2, 3, 7, 6]],
        [transformed_corners[i] for i in [1, 2, 6, 5]],
        [transformed_corners[i] for i in [0, 3, 7, 4]],
    ]

    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        points[:, 0],
        points[:, 1],
        points[:, 2],
        color="b",
        marker="o",
        s=10,
        label="Points",
    )

    # Draw the bounding box
    box = Poly3DCollection(edges, edgecolor="r", alpha=0.2, linewidths=1.5)
    ax.add_collection3d(box)

    # Plot the center of the OBB
    ax.scatter(*center, color="r", marker="x", s=50, label="OBB Center")

    # Labeling
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.title("Oriented Bounding Box and Points")
    plt.show()


if __name__ == "__main__":
    # Example data and bounding box calculation

    # Define the center point around which points will be generated
    center_point = np.array([2.0, 3.0, 3.0])

    # Define the number of points and standard deviation for Gaussian randomness
    num_points = 50
    std_dev = 0.5

    # Generate points with Gaussian randomness around the center point
    points = np.random.normal(loc=center_point, scale=std_dev, size=(num_points, 3))

    center, size, rotation_matrix = oriented_bounding_box(points)

    # Plot the points and the oriented bounding box
    plot_oriented_bounding_box(points, center, size, rotation_matrix)
