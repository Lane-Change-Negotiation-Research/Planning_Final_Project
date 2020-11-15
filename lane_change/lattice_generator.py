"""Lattice Generator will generate lattice graphs"""

import math

import numpy as np
import carla
from shapely.geometry import LineString

from planning_utils import slvt_to_cartesian, sample_trajectory, get_x_y_v_trajectory
from path_optimizer import PathOptimizer


class LatticeGenerator:
    def __init__(self, center_line_linestring: LineString):

        self.linestring = center_line_linestring
        self.arc_length = self.linestring.length
        self.path_optimizer = PathOptimizer()

        self.s_l_lattice = self._generate_s_l_lattice(1, 60, 5)

    def sample_smooth_trajectory(self):

        ## Get smooth path in s-l frame

        paths = []
        path_validity = []

        # traj_x, traj_y, traj_v = get_x_y_v_trajectory(
        #     sample_trajectory(max_v=18), self.s_l_lattice
        # )

    def _generate_s_l_lattice(
        self,
        span_l: float,
        num_samples_s: float,
        num_samples_l: float,
    ):

        lattice = []

        for s in np.linspace(0, self.arc_length, num_samples_s):

            x_coords = []
            y_coords = []

            for l in np.linspace(-span_l, span_l, num_samples_l):
                lattice_pt = slvt_to_cartesian(self.linestring, s, l)
                x_coords.append(lattice_pt[0])
                y_coords.append(lattice_pt[1])

            lattice.append([x_coords, y_coords])

        return lattice
