import random
import copy

from scipy import optimize
import numpy as np
from shapely.geometry import LineString


class ComputeCurvature:
    """
    Computes the curvature for a point through the circle fitting method.
    TLDR: Take a bunch of points around and including the point of interest. Fit a circle on these points.
          Inverse of the radius of the fitted circle is the curvature.
    """

    def __init__(self):
        """ Initialize some variables """
        self.xc = 0  # X-coordinate of circle center
        self.yc = 0  # Y-coordinate of circle center
        self.r = 0  # Radius of the circle
        self.xx = np.array([])  # Data points
        self.yy = np.array([])  # Data points

    def calc_r(self, xc, yc):
        """ calculate the distance of each 2D points from the center (xc, yc) """
        return np.sqrt((self.xx - xc) ** 2 + (self.yy - yc) ** 2)

    def f(self, c):
        """ calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
        ri = self.calc_r(*c)
        return ri - ri.mean()

    def df(self, c):
        """Jacobian of f_2b
        The axis corresponding to derivatives must be coherent with the col_deriv option of leastsq"""
        xc, yc = c
        df_dc = np.empty((len(c), self.xx.size))

        ri = self.calc_r(xc, yc)
        df_dc[0] = (xc - self.xx) / ri  # dR/dxc
        df_dc[1] = (yc - self.yy) / ri  # dR/dyc
        df_dc = df_dc - df_dc.mean(axis=1)[:, np.newaxis]
        return df_dc

    def fit(self, xx, yy):
        self.xx = xx
        self.yy = yy
        center_estimate = np.r_[np.mean(xx), np.mean(yy)]
        center = optimize.leastsq(
            self.f, center_estimate, Dfun=self.df, col_deriv=True
        )[0]

        self.xc, self.yc = center
        ri = self.calc_r(*center)
        self.r = ri.mean()

        return 1 / self.r  # Return the curvature


def slvt_to_cartesian(
    linestring: LineString, s: float, d: float, resolution: float = 0.001
):
    """
    Get the cartesian coordinates from a linestring that represents the lane center-line, the s (arc length), and d (lateral distance).
    Returns the cartesian x-coordinate, the cartesian y-coordinate and the theta (heading/yaw) in cartesian frame.
    """

    point_on_linestring = linestring.interpolate(s)

    # Find heading direction of the lane LineString by using a small section of the LineString,
    # around point_on_linestring computed above.
    local_section_of_spline_start = linestring.interpolate(max(0, s - resolution))
    local_section_of_spline_end = linestring.interpolate(
        min(linestring.length, s + resolution)
    )
    local_section_heading_in_cartesian_coordinates = np.degrees(
        np.arctan2(
            local_section_of_spline_end.y - local_section_of_spline_start.y,
            local_section_of_spline_end.x - local_section_of_spline_start.x,
        )
    )

    # Get the cartesian point at offset by d, along a direction perpendicular to the heading of the local section of the lane LineString.
    angle_to_extend = (
        (local_section_heading_in_cartesian_coordinates + 90) * np.pi / 180
    )
    cartesian_point = [
        point_on_linestring.x + d * np.cos(angle_to_extend),
        point_on_linestring.y + d * np.sin(angle_to_extend),
    ]

    # Get the heading in cartesian frame.
    cartesian_heading = local_section_heading_in_cartesian_coordinates * np.pi / 180

    return cartesian_point[0], cartesian_point[1], cartesian_heading


def sample_trajectory(
    start_state=[2, 0, 3],
    delta_y_set=[-1, 0, 1],
    delta_v_set=[1, 0, -1],  # actions
    max_y=4,
    min_y=0,
    max_x=60,
    max_v=24,
    min_v=3,
):

    """
    Sample a trajectory for the given lattice.
    """

    trajectory = []

    current = copy.deepcopy(start_state)  # [2, 0, 3]  # y, x, v

    trajectory.append(tuple(current))

    for i in range(1, max_x):

        dy = random.choice(delta_y_set)
        dv = random.choice(delta_v_set)

        next_y = min(max(current[0] + dy, min_y), max_y)
        next_v = min(max(current[2] + dv, min_v), max_v)

        trajectory.append((next_y, i, next_v))

        current = [next_y, i, next_v]

    return trajectory


def get_x_y_v_trajectory(s_l_trajectory, lattice):

    x_traj = []
    y_traj = []
    v_traj = []

    for i in range(len(s_l_trajectory)):

        y = s_l_trajectory[i][0]
        v = s_l_trajectory[i][2]

        x_cartesian = lattice[i][0][y]
        y_cartesian = lattice[i][1][y]

        x_traj.append(x_cartesian)
        y_traj.append(y_cartesian)
        v_traj.append(v)

    return x_traj, y_traj, v_traj
