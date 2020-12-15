""" Trajectory Planner will take the action assigned by path selector and generate trajectory to follow.
    At end of trajectory, request replan from Lattice Generator. """

import math
import copy
import carla
from enum import IntEnum
from util import get_ego_waypoint


class action(IntEnum):
    ACCELERATE = 0
    CONSTANT_SPEED = 1
    DECELERATE = 2
    SWITCH_LANE = 3


class VecTemp:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def norm(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def add(self, other):
        return VecTemp(self.x + other.x, self.y + other.y)

    def sub(self, other):
        return VecTemp(self.x - other.x, self.y - other.y)

    def dot(self, other):
        upper = self.x * other.x + self.y + other.y
        lower = self.norm() * other.norm()
        if lower <= 0.00001:
            return 1
        return upper / lower


class PoseTemp:
    def __init__(self, x=0, y=0, theta=0, speed=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.speed = speed

    def wrapToPi(self, theta):
        return (theta + math.pi) % (2.0 * math.pi) - math.pi

    def distance(self, pose):
        return math.sqrt((self.x - pose.x) ** 2.0 + (self.y - pose.y) ** 2.0)

    def add(self, pose):
        new_pose = PoseTemp()
        new_pose.x = (
            pose.x + self.x * math.cos(pose.theta) - self.y * math.sin(pose.theta)
        )
        new_pose.y = (
            pose.y + self.x * math.sin(pose.theta) + self.y * math.cos(pose.theta)
        )
        new_pose.theta = self.wrapToPi(self.theta + pose.theta)
        new_pose.speed = self.speed
        return new_pose

    def vecTo(self, pose):
        new_vec = VecTemp()
        new_vec.x = pose.x - self.x
        new_vec.y = pose.y - self.y
        return new_vec

    def vecFromTheta(self):
        return VecTemp(math.cos(self.theta), math.sin(self.theta))

    def isInfrontOf(self, pose):
        diff_vec = pose.vecTo(self)
        other_vec = pose.vecFromTheta()
        return diff_vec.dot(other_vec) > 0

    def scalarMultiply(self, scalar):
        new_pose = PoseTemp()
        new_pose.x = self.x * scalar
        new_pose.y = self.y * scalar
        new_pose.theta = self.theta * scalar


# The lattice generator generates plans three moves ahead
# Check collision for all plans
# Pick plan to follow, favoring previous plan
# Execute one step and replan

# I need a simple lattice generator
# Another detailed trajectory planner


# class PlanGenerator:
#     def __init__(self):
#         x = 0

#     def get_next_actions(self, k=10):

#         return [
#             action.ACCELERATE,
#             # action.ACCELERATE,
#             # action.ACCELERATE,
#             # action.ACCELERATE,
#             action.ACCELERATE,
#             action.CONSTANT_SPEED,
#             action.ACCELERATE,
#             action.DECELERATE,
#             action.ACCELERATE,
#             action.ACCELERATE,
#             action.ACCELERATE,
#             action.ACCELERATE,
#             action.ACCELERATE,
#             action.SWITCH_LANE,
#         ]


class PathFollower:
    def __init__(self, world, ego_vehicle, time_step):

        self.SAME_POSE_THRESHOLD = 2
        self.SAME_POSE_LOWER_THRESHOLD = 0.02

        # self.plan_generator = PlanGenerator()
        self.trajectory_generator = TrajGenerator(world, ego_vehicle, time_step)

    # def _generate_plan(self):

    #     return self.plan_generator.get_next_actions()

    def get_trajectory(self, starting_speed, starting_waypoint, next_actions):

        # next_actions = self._generate_plan()

        concatenated_trajectory = []
        lane_switch_flags = []

        speed = copy.deepcopy(starting_speed)
        waypoint = starting_waypoint

        for next_action in next_actions:

            if next_action == action.CONSTANT_SPEED:
                traj_poses, last_waypoint = self.trajectory_generator.constTraj(
                    speed, waypoint
                )
                waypoint = last_waypoint
            elif next_action == action.ACCELERATE:
                traj_poses, last_waypoint = self.trajectory_generator.accTraj(
                    speed, waypoint
                )
                speed = traj_poses[-1].speed
                waypoint = last_waypoint
            elif next_action == action.DECELERATE:
                traj_poses, last_waypoint = self.trajectory_generator.decTraj(
                    speed, waypoint
                )
                speed = traj_poses[-1].speed
                waypoint = last_waypoint
            elif next_action == action.SWITCH_LANE:
                traj_poses, last_waypoint = self.trajectory_generator.laneChangeTraj(
                    speed, waypoint
                )
                speed = traj_poses[-1].speed
                waypoint = last_waypoint

            concatenated_trajectory += traj_poses
            if next_action == action.SWITCH_LANE:
                lane_switch_flags += [1 for i in range(len(traj_poses))]
            else:
                lane_switch_flags += [0 for i in range(len(traj_poses))]

        return concatenated_trajectory, lane_switch_flags

    def findNextLanePose(self, cur_vehicle_pose, traj_poses):
        way_pose = traj_poses[0]

        for i, way_pose in enumerate(traj_poses):

            if (
                way_pose.isInfrontOf(cur_vehicle_pose)
                and way_pose.distance(cur_vehicle_pose) > self.SAME_POSE_LOWER_THRESHOLD
            ):
                return way_pose, i
        return way_pose, 0


class TrajGenerator:
    # SAME_POSE_THRESHOLD = 2
    # SAME_POSE_LOWER_THRESHOLD = 0.02

    def __init__(self, world, ego_vehicle, time_step):
        self.world = world
        self.time_step = time_step
        # self.ego = ego_vehicle
        # self.start_speed = speed  # m/s. The designed speed of starting lattice node instead of true speed
        self.a_max = 5  # m/s^2. Designed maximum acc
        self.jerk_time = 0.4  # sec. Time to go from 0 acc to a_max or a_min
        # self.current_action = action.NO_ACTION
        self.traj_time = 1  # sec. acc/dec/const time. Must be >= 2*jerk_time
        self.lane_width = get_ego_waypoint(world, ego_vehicle).lane_width
        self.lane_change_time = 4.5  # sec. Based on https://toledo.net.technion.ac.il/files/2012/12/TRR_ToledoZohar_07.pdf between 4~5 seconds
        self.lane_change_time_disc = (
            time_step  # One pose per time disc for generated trajectories
        )
        # self.lane_change_length = math.sqrt((speed * self.lane_change_time_constant)**2. - self.lane_width**2.) # based on time and speed
        # self.lane_change_length = self.start_speed # Heuristic that approximates human lane change distance based on https://www.mchenrysoftware.com/board/viewtopic.php?t=339

    def constTraj(self, starting_speed, starting_waypoint):
        k = int(1 / self.time_step)
        starting_speed = starting_speed / 3.6  # Change from km/hr to m/s
        sampling_radius = (
            starting_speed * self.traj_time / k
        )  # Depends on distance to travel and number of points
        traj_poses = []
        last_waypoint = starting_waypoint

        for _ in range(k):
            next_waypoints = last_waypoint.next(sampling_radius)

            if len(next_waypoints) == 0:
                break
            else:
                # only one option available ==> lanefollowing
                next_waypoint = next_waypoints[0]
                next_pose = next_waypoint.transform.location
                next_pose = PoseTemp(next_pose.x, next_pose.y, 0, starting_speed * 3.6)

            traj_poses.append(next_pose)
            last_waypoint = next_waypoint

        # Returns the trajectory poses and the final waypoint for the next trajectory
        return traj_poses, last_waypoint

    def accTraj(self, starting_speed, starting_waypoint):
        k = int(1 / self.time_step)
        starting_speed = starting_speed / 3.6  # Change from km/hr to m/s
        last_waypoint = starting_waypoint
        traj_poses = []
        jerk_k = (
            k * self.jerk_time / self.traj_time
        )  # Number of poses during constant jerk
        dt = self.traj_time / k
        last_d = 0

        for i in range(k):
            # Trapezoidal acceleration profile
            if i == 0:
                t0 = 0
                d0 = 0
                v0 = starting_speed
                a0 = 0
                j = self.a_max / self.jerk_time
            elif i == jerk_k:
                t0 = self.jerk_time
                d0 = 1 / 6 * j * (self.jerk_time ** 3) + v0 * self.jerk_time
                v0 = 1 / 2 * j * (self.jerk_time ** 2) + v0
                a0 = self.a_max
                j = 0
            elif i == k - jerk_k:
                t_const_a = self.traj_time - 2 * self.jerk_time
                t0 = (k - jerk_k) * dt  # = t0 + t_const_a
                d0 = 1 / 2 * a0 * t_const_a ** 2 + v0 * t_const_a + d0
                v0 = self.a_max * t_const_a + v0
                a0 = self.a_max
                j = -self.a_max / self.jerk_time

            t = (i + 1) * dt - t0
            next_v = 1 / 2 * j * t ** 2 + a0 * t + v0
            next_d = 1 / 6 * j * t ** 3 + 1 / 2 * a0 * t ** 2 + v0 * t + d0
            delta_d = next_d - last_d
            last_d = next_d

            # Get the next waypoint and append it to traj_poses
            next_waypoints = last_waypoint.next(delta_d)
            if len(next_waypoints) == 0:
                break
            else:
                # only one option available ==> lanefollowing
                next_waypoint = next_waypoints[0]
                next_pose = next_waypoint.transform.location
                next_pose = PoseTemp(next_pose.x, next_pose.y, 0, next_v * 3.6)

            traj_poses.append(next_pose)
            last_waypoint = next_waypoint

        return traj_poses, last_waypoint

    def decTraj(self, starting_speed, starting_waypoint):
        k = int(1 / self.time_step)
        starting_speed = starting_speed / 3.6  # Change from km/hr to m/s
        last_waypoint = starting_waypoint
        traj_poses = []
        jerk_k = (
            k * self.jerk_time / self.traj_time
        )  # Number of poses during constant jerk
        dt = self.traj_time / k
        last_d = 0

        for i in range(k):
            # Trapezoidal deceleration profile
            if i == 0:
                t0 = 0
                d0 = 0
                v0 = starting_speed
                a0 = 0
                j = -self.a_max / self.jerk_time
            elif i == jerk_k:
                t0 = self.jerk_time
                d0 = 1 / 6 * j * (self.jerk_time ** 3) + v0 * self.jerk_time
                v0 = 1 / 2 * j * (self.jerk_time ** 2) + v0
                a0 = -self.a_max
                j = 0
            elif i == k - jerk_k:
                t_const_a = self.traj_time - 2 * self.jerk_time
                t0 = (k - jerk_k) * dt  # = t0 + t_const_a
                d0 = 1 / 2 * a0 * t_const_a ** 2 + v0 * t_const_a + d0
                v0 = self.a_max * t_const_a + v0
                a0 = -self.a_max
                j = -self.a_max / self.jerk_time

            t = (i + 1) * dt - t0
            next_v = 1 / 2 * j * t ** 2 + a0 * t + v0
            next_d = 1 / 6 * j * t ** 3 + 1 / 2 * a0 * t ** 2 + v0 * t + d0
            delta_d = next_d - last_d
            last_d = next_d

            # Get the next waypoint and append it to traj_poses
            next_waypoints = last_waypoint.next(delta_d)
            if len(next_waypoints) == 0:
                break
            else:
                # only one option available ==> lanefollowing
                next_waypoint = next_waypoints[0]
                next_pose = next_waypoint.transform.location
                next_pose = PoseTemp(next_pose.x, next_pose.y, 0, next_v * 3.6)

            traj_poses.append(next_pose)
            last_waypoint = next_waypoint

        return traj_poses, last_waypoint

    def laneChangeTraj(self, starting_speed, starting_waypoint):
        neutral_traj = self.cubicSplineGen(starting_speed)
        starting_pose = starting_waypoint.transform.location
        starting_z = starting_pose.z
        starting_pose = PoseTemp(starting_pose.x, starting_pose.y, 0, starting_speed)

        traj_poses = []

        for pose in neutral_traj:
            next_pose = pose.add(starting_pose)
            traj_poses.append(next_pose)

        final_location = carla.Location(next_pose.x, next_pose.y, starting_z)
        last_waypoint = self.world.get_map().get_waypoint(final_location)
        return traj_poses, last_waypoint

    def cubicSplineGen(self, v_cur):
        # determine external parameters
        w = self.lane_width
        l = v_cur  # lane_change_length. Heuristic that approximates human lane change distance based on https://www.mchenrysoftware.com/board/viewtopic.php?t=339
        tf = self.lane_change_time
        # tf = math.sqrt(l**2 + w**2) * r / v_cur

        v_cur = v_cur / 3.6  # km/hr to m/s
        if v_cur < 5:
            v_cur = 5

        # parameters for x
        dx = 0
        cx = v_cur
        ax = (2.0 * v_cur * tf - 2.0 * l) / (tf ** 3.0)
        bx = -3.0 / 2 * ax * tf

        # parameters for y
        dy = 0
        cy = 0
        ay = -2.0 * w / (tf ** 3.0)
        by = 3 * w / (tf ** 2.0)

        # return result
        neutral_traj = []

        # time loop
        time_disc = self.lane_change_time_disc
        total_loop_count = int(tf / time_disc + 1)
        for i in range(1, total_loop_count):
            t = i * time_disc
            x_value = ax * (t ** 3.0) + bx * (t ** 2.0) + cx * t + dx
            y_value = -(ay * (t ** 3.0) + by * (t ** 2.0) + cy * t + dy)
            x_deriv = 3.0 * ax * (t ** 2.0) + 2.0 * bx * t + cx
            y_deriv = -(3.0 * ay * (t ** 2.0) + 2.0 * by * t + cy)
            theta = math.atan2(y_deriv, x_deriv)
            speed = math.sqrt(y_deriv ** 2.0 + x_deriv ** 2.0)
            pose = PoseTemp()
            pose.speed = speed * 3.6
            pose.x = x_value
            pose.y = y_value
            pose.theta = theta
            neutral_traj.append(pose)

        return neutral_traj

    def replan_request(self):
        pass