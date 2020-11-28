""" Trajectory Planner will take the action assigned by path selector and generate trajectory to follow.
    At end of trajectory, request replan from Lattice Generator. """

import math
import copy
from enum import Enum


class action(Enum):
    NO_ACTION = 0
    CONSTANT_SPEED = 1
    ACCELERATE = 2
    DECELERATE = 3
    SWITCH_LANE = 4


def convert_list_of_waypoints_to_list_of_pose(waypoints_list):

    out = []

    for wp in waypoints_list:
        location = wp.transform.location
        rotation = wp.transform.rotation
        out.append(PoseTemp(x=location.x, y=location.y, theta=rotation.yaw))

    return out


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


class TrajGenerator:
    def __init__(self, current_lane_waypoints):

        self.SAME_POSE_THRESHOLD = 2.2
        self.SAME_POSE_LOWER_THRESHOLD = 0.02

        self.current_lane_waypoints = current_lane_waypoints
        self.current_lane_pose_array = convert_list_of_waypoints_to_list_of_pose(
            self.current_lane_waypoints
        )
        self.lane_width = self.current_lane_waypoints[0].lane_width
        self.lane_change_time = 4.5  # sec. Based on https://toledo.net.technion.ac.il/files/2012/12/TRR_ToledoZohar_07.pdf between 4~5 seconds
        self.lane_change_time_disc = (
            0.1  # One pose per time disc for generated trajectories
        )
        self.traj_time = 1

        # Placeholders
        self.current_action = action.NO_ACTION
        self.generated_path = []
        self.path_pointer = 0
        self.action_start_time = 0

        self.reset()

    def reset(self, cur_act=action.NO_ACTION):
        self.current_action = cur_act
        self.generated_path = []
        self.path_pointer = 0
        self.action_start_time = 0

    def constTraj(self, starting_speed, starting_waypoint, k=10):
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
                next_pose = PoseTemp(next_pose.x, next_pose.y, 0, starting_speed)

            traj_poses.append(next_pose)
            last_waypoint = next_waypoint

        # Returns the trajectory poses and the final waypoint for the next trajectory
        return traj_poses, last_waypoint

    def findNextLanePose(self, cur_vehicle_pose, lane_pose_array):
        closest_pose = lane_pose_array[0]

        for lane_waypoint in lane_pose_array:
            closest_pose = lane_waypoint
            way_pose = PoseTemp(lane_waypoint.pose)
            if (
                way_pose.distance(cur_vehicle_pose) < self.SAME_POSE_THRESHOLD
                and way_pose.isInfrontOf(cur_vehicle_pose)
                and way_pose.distance(cur_vehicle_pose) > self.SAME_POSE_LOWER_THRESHOLD
            ):
                return closest_pose
        return closest_pose

    def laneChangeTraj(
        self, current_vehicle_transform, current_speed, first_call=False
    ):

        current_vehicle_pose = PoseTemp(
            x=current_vehicle_transform.location.x,
            y=current_vehicle_transform.location.y,
            theta=current_vehicle_transform.rotation.yaw,
            speed=current_speed,
        )

        if first_call:
            neutral_traj = self.cubicSplineGen(current_speed)

            closest_pose = None
            for lane_wp_pose in self.current_lane_pose_array:
                closest_pose = copy.deepcopy(lane_wp_pose)
                if (
                    closest_pose.distance(current_vehicle_pose)
                    < self.SAME_POSE_THRESHOLD
                    and closest_pose.isInfrontOf(current_vehicle_pose)
                    and closest_pose.distance(current_vehicle_pose)
                    > self.SAME_POSE_LOWER_THRESHOLD
                ):
                    break

            if len(self.generated_path) == 0:
                for pose in neutral_traj:
                    self.generated_path.append(pose.add(closest_pose))

            self.path_pointer = 0

        while self.path_pointer < len(self.generated_path):
            # traj pose
            pose = self.generated_path[self.path_pointer]

            if (
                pose.isInfrontOf(current_vehicle_pose)
                and pose.distance(current_vehicle_pose) > self.SAME_POSE_LOWER_THRESHOLD
            ):
                break

            self.path_pointer += 1

        # self.path_pointer = min(self.path_pointer, len(self.generated_path) - 1)
        action_progress = float(self.path_pointer) / len(self.generated_path)

        end_of_action = False
        if action_progress >= 0.99:
            end_of_action = True
            action_progress = 1.0

        # Plan to send
        plan = {}

        if end_of_action:
            self.reset()
            plan["tracking_pose"] = None
        else:
            plan["tracking_pose"] = self.generated_path[self.path_pointer]

        plan["end_of_action"] = end_of_action
        plan["action_progress"] = action_progress

        # future poses
        path_pointer = 0  # copy.deepcopy(self.path_pointer)
        plan["future_poses"] = []
        while path_pointer < len(self.generated_path):
            new_pose = PoseTemp()
            new_pose.x = self.generated_path[path_pointer].x
            new_pose.y = self.generated_path[path_pointer].y
            new_pose.theta = self.generated_path[path_pointer].theta
            plan["future_poses"].append(new_pose)
            path_pointer += 1

        print("Len:", len(self.generated_path), "Pointer:", self.path_pointer)
        # self.reset()

        return plan

    def cubicSplineGen(self, v_cur):
        v_cur = v_cur / 3.6  # km/hr to m/s
        if v_cur < 5:
            v_cur = 5
        # determine external parameters
        w = self.lane_width
        l = v_cur * 3.6  # In km/h
        tf = self.lane_change_time
        # tf = math.sqrt(l**2 + w**2) * r / v_cur

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
        for i in range(total_loop_count):
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