import matplotlib
matplotlib.use('Agg')

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.spatial.transform import Rotation
from scipy.optimize import root_scalar

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Find the maximum value of a B-spline between t_start and t_end.
def find_maximum_bspl(spline, t_start, t_end, num_points=1000):
    t_values = np.linspace(t_start, t_end, num_points)
    spline_values = spline(t_values)
    abs_spline_values = np.abs(spline_values)
    max_value_index = np.argmax(abs_spline_values)
    max_abs_value = abs_spline_values[max_value_index]
    max_time = t_values[max_value_index]

    derivative_spline = spline.derivative()

    def derivative_root_finder(t):
        return derivative_spline(t)

    critical_points = []
    for i in range(num_points - 1):
        if t_values[i] < t_start or t_values[i + 1] > t_end:
            continue
        try:
            root_result = root_scalar(
                derivative_root_finder, bracket=[t_values[i], t_values[i + 1]], method='brentq'
            )
            if root_result.converged and t_start <= root_result.root <= t_end:
                critical_points.append(root_result.root)
        except ValueError:
            pass

    evaluation_points = [t_start, t_end] + critical_points
    for t in evaluation_points:
        spline_value = spline(t)
        abs_spline_value = np.abs(spline_value)
        if abs_spline_value > max_abs_value:
            max_abs_value = abs_spline_value
            max_time = t

    return max_abs_value, max_time

def make_interp_spline_with_constraints(x, y, k=3, t=None, bc_type=None, v_max=None, a_max=None):
    spline_pos = make_interp_spline(x=x, y=y, k=k, t=t, bc_type=bc_type)

    if v_max is None or a_max is None:
        return spline_pos, x

    spline_vel = spline_pos.derivative()
    spline_acc = spline_vel.derivative()
    factors = []
    for i in range(len(x) - 1):
        factors.append(max(find_maximum_bspl(spline_vel, x[i], x[i+1])[0]/v_max, (find_maximum_bspl(spline_acc, x[i], x[i+1])[0]/a_max)**0.5))
    
    x_new = [x[0]]
    for i in range(len(factors)):
        x_new.append(x_new[-1] + (x[i + 1] - x[i]) * factors[i])

    return make_interp_spline(x=x, y=y, k=k, t=t, bc_type=bc_type), x_new

def create_path(x, y, z, theta_x, theta_y, theta_z, max_velocity = 1, max_acceleration = 1, max_angular_velocity = 1, max_angular_acceleration = 1):
    orientations = Rotation.from_euler('xyz', np.column_stack((theta_x, theta_y, theta_z)), degrees=True).as_euler('xyz', degrees=True)
    dim_vars = (x, y, z)

    # Fit splines to x y z as a function of t
    splines = []
    new_xs = [] # Spline intervals that ensure maxes are not exceeded
    for var in dim_vars:
        spline, new_x = make_interp_spline_with_constraints(
            # Example, max absolute 1 m/s velocity and 1 m/s^2 acceleration 
            np.linspace(0, 1, len(x)), var, k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)]), v_max=max_velocity, a_max=max_acceleration
        )
        splines.append(spline)
        new_xs.append(new_x)

    # Define modified rotation spline (interpolates on rot_vecs)
    class RotationSpline:
        def __init__(self, t, quaternions, v_max=None, a_max=None):

            assert len(t) == len(quaternions)
            
            self.t = t
            self.quaternions = quaternions

            self.rotations = Rotation.from_quat(quaternions)
            self.rotvecs = self.rotations.as_rotvec()

            self.v_max = v_max
            self.a_max = a_max

            self.spline_x, self.t_x = make_interp_spline_with_constraints(t, self.rotvecs[:, 0], k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)]), v_max=self.v_max, a_max=self.a_max)
            self.spline_y, self.t_y = make_interp_spline_with_constraints(t, self.rotvecs[:, 1], k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)]), v_max=self.v_max, a_max=self.a_max)
            self.spline_z, self.t_z = make_interp_spline_with_constraints(t, self.rotvecs[:, 2], k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)]), v_max=self.v_max, a_max=self.a_max)

        def __call__(self, t_single):
            t_array = np.array([t_single])
            quaternions_fine = self.evaluate(t_array)
            return quaternions_fine[0]
        
        def update_t(self, new_t):
            self.spline_x, self.t_x = make_interp_spline_with_constraints(new_t, self.rotvecs[:, 0], k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)]), v_max=self.v_max, a_max=self.a_max)
            self.spline_y, self.t_y = make_interp_spline_with_constraints(new_t, self.rotvecs[:, 1], k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)]), v_max=self.v_max, a_max=self.a_max)
            self.spline_z, self.t_z = make_interp_spline_with_constraints(new_t, self.rotvecs[:, 2], k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)]), v_max=self.v_max, a_max=self.a_max)

        def evaluate(self, t_fine):
            rotvecs_fine = np.vstack((
                self.spline_x(t_fine),
                self.spline_y(t_fine),
                self.spline_z(t_fine)
            )).T
            quaternions_fine = Rotation.from_rotvec(rotvecs_fine).as_quat()
            return quaternions_fine

        def as_euler(self, t_fine, order='xyz', degrees=False):
            quaternions_fine = self.evaluate(t_fine)
            rotations = Rotation.from_quat(quaternions_fine)
            return rotations.as_euler(order, degrees=degrees)

        def angular_velocity(self, t_fine):
            angular_vel_x = self.spline_x(t_fine, 1)
            angular_vel_y = self.spline_y(t_fine, 1)
            angular_vel_z = self.spline_z(t_fine, 1)
            return np.vstack((angular_vel_x, angular_vel_y, angular_vel_z)).T

        def angular_acceleration(self, t_fine):
            angular_acc_x = self.spline_x(t_fine, 2)
            angular_acc_y = self.spline_y(t_fine, 2)
            angular_acc_z = self.spline_z(t_fine, 2)
            return np.vstack((angular_acc_x, angular_acc_y, angular_acc_z)).T
        
    # Fit splines to theta_x theta_y theta_z as a function of t
    quaternion_spline = RotationSpline(np.linspace(0, 1, len(x)), Rotation.from_euler("xyz", orientations, degrees=True).as_quat(), v_max=max_angular_velocity, a_max=max_angular_acceleration)

    new_xs.extend([quaternion_spline.t_x, quaternion_spline.t_y, quaternion_spline.t_z])

    final_x = [0.0] # Create final time interval such that each individual interval is max of corresponding intervals for each spline
    for i in range(len(new_xs[0]) - 1):
        final_x.append(final_x[i] + max(new_x[i+1] - new_x[i] for new_x in new_xs))

    # Final update to splines with final_x
    splines = [
        make_interp_spline_with_constraints(
            final_x, var, k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)]), v_max=max_velocity, a_max=max_acceleration
        )[0] for var in dim_vars
    ]
    quaternion_spline.update_t(final_x)

    # Define a finer grid of t values for evaluation
    t_fine = np.linspace(0, final_x[-1], 100)

    # For angular velocities and accelerations:
    angular_velocities = quaternion_spline.angular_velocity(t_fine)
    angular_accelerations = quaternion_spline.angular_acceleration(t_fine)

    # Compute positions
    positions = [spline(t_fine) for spline in splines]
    x_fine, y_fine, z_fine = positions
    orientations = quaternion_spline.as_euler(t_fine, order='xyz', degrees=True)

    # Compute velocities (first derivative)
    velocities = [spline(t_fine, 1) for spline in splines]
    v_x, v_y, v_z = velocities
    angular_velocities = quaternion_spline.angular_velocity(t_fine) #orientation_spline(t_fine, 1)

    # Compute accelerations (second derivative)
    accelerations = [spline(t_fine, 2) for spline in splines]
    a_x, a_y, a_z = accelerations
    angular_accelerations = quaternion_spline.angular_acceleration(t_fine) #orientation_spline(t_fine, 2)

    return positions, velocities, accelerations, orientations, angular_velocities, angular_accelerations
