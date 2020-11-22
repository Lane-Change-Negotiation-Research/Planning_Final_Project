
""" Trajectory Planner will take the action assigned by path selector and generate trajectory to follow.
    At end of trajectory, request replan from Lattice Generator. """

import Queue
import math
import copy
from enum import Enum


class action(Enum):
    NO_ACTION = 0
    CONSTANT_SPEED = 1
    ACCELERATE = 2
    DECELERATE = 3
    SWITCH_LANE = 4


class VecTemp:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def norm(self):
        return math.sqrt(self.x ** 2 + self.y**2)

    def add(self, other):
        return VecTemp(self.x+other.x, self.y+other.y)

    def sub(self, other):
        return VecTemp(self.x-other.x, self.y-other.y)

    def dot(self, other):
        upper = self.x*other.x + self.y+other.y
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
        return (theta + math.pi) % (2. * math.pi) - math.pi

    def distance(self, pose):
        return math.sqrt((self.x - pose.x) ** 2. + (self.y - pose.y) ** 2.)

    def add(self, pose):
        new_pose = PoseTemp()
        new_pose.x = pose.x + self.x * \
            math.cos(pose.theta) - self.y * math.sin(pose.theta)
        new_pose.y = pose.y + self.x * \
            math.sin(pose.theta) + self.y * math.cos(pose.theta)
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


class TrajGenerator:
    SAME_POSE_THRESHOLD = 2
    SAME_POSE_LOWER_THRESHOLD = 0.02

    def __init__(self, speed, waypoint, current_lane_waypoints, right_lane_waypoints):
        self.speed = speed  # m/s
        self.waypoint = waypoint
        self.current_action = action.NO_ACTION
        self.current_lane_waypoints = current_lane_waypoints
        # self.right_lane_waypoints = right_lane_waypoints
        self.lane_width = waypoint.lane_width
        self.lane_change_time = 4.5  # sec. Based on https://toledo.net.technion.ac.il/files/2012/12/TRR_ToledoZohar_07.pdf between 4~5 seconds
        self.lane_change_time_disc = 0.1  # One pose per time disc for generated trajectories
        # self.lane_change_length = math.sqrt((speed * self.lane_change_time_constant)**2. - self.lane_width**2.) # based on time and speed
        self.lane_change_length = self.speed # Heuristic that approximates human lane change distance based on https://www.mchenrysoftware.com/board/viewtopic.php?t=339

    def findNextLanePose(self, cur_vehicle_pose, lane_pose_array):
        closest_pose = lane_pose_array[0]

        for lane_waypoint in lane_pose_array:
            closest_pose = lane_waypoint
            way_pose = PoseTemp(lane_waypoint.pose)
            if way_pose.distance(cur_vehicle_pose) < TrajGenerator.SAME_POSE_THRESHOLD and\
                    way_pose.isInfrontOf(cur_vehicle_pose) and \
                way_pose.distance(cur_vehicle_pose) > TrajGenerator.SAME_POSE_LOWER_THRESHOLD:
                return closest_pose
        return closest_pose

    def laneChangeTraj(self):
        neutral_traj = self.cubicSplineGen(self.speed)
        
        pass

    def cubicSplineGen(self, v_cur):
        v_cur = v_cur/3.6 # km/hr to m/s
        if v_cur < 5:
            v_cur = 5
        # determine external parameters
        w = self.lane_width
        l = self.lane_change_length
        tf = self.lane_change_time
        # tf = math.sqrt(l**2 + w**2) * r / v_cur

        # parameters for x
        dx = 0
        cx = v_cur
        ax = (2.*v_cur * tf - 2.*l) / (tf ** 3.)
        bx = -3./2 * ax * tf

        # parameters for y
        dy = 0
        cy = 0
        ay = -2.*w/(tf ** 3.)
        by = 3*w/(tf ** 2.)

        # return result
        neutral_traj = []

        # time loop
        time_disc = self.lane_change_time_disc
        total_loop_count = int(tf / time_disc + 1)
        for i in range(total_loop_count):
            t = i * time_disc
            x_value = ax*(t**3.)+bx*(t**2.)+cx*t+dx
            y_value = -(ay*(t**3.)+by*(t**2.)+cy*t+dy)
            x_deriv = 3.*ax*(t**2.)+2.*bx*t+cx
            y_deriv = -(3.*ay*(t**2.)+2.*by*t+cy)
            theta = math.atan2(y_deriv, x_deriv)
            speed = math.sqrt(y_deriv**2.+x_deriv**2.)
            pose = PoseTemp()
            pose.speed = speed*3.6
            pose.x = x_value
            pose.y = y_value
            pose.theta = theta
            neutral_traj.append(pose)

        return neutral_traj

    def replan_request(self):
        pass