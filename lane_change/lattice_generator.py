
""" Lattice Generator will generate lattice graphs for the collision checker """

import numpy as np
import math
import carla


class LatticeGenerator:
	num_waypoints = 5

	def __init__(self, speed, waypoint, current_lane_waypoints, right_lane_waypoints):
		self.speed = speed  # m/s
		self.waypoint = waypoint
		# self.current_lane_waypoints = current_lane_waypoints
		# self.right_lane_waypoints = right_lane_waypoints
		self.lane_width = waypoint.lane_width
		self.lane_change_time_constant = 4.5  # sec
		self.lane_change_length = math.sqrt(
		    (speed * self.lane_change_time_constant)**2. - self.lane_width**2.)  # based on time and speed
		self.motionPrimitives = []

	def generateMotionPrimitives(self):
		pass

	def laneChangePrimitive(self):
		pass

	def acceleratePrimitive(self):
		pass

	def deceleratePrimitive(self):
		pass

	def constSpeedPrimitive(self):
		pass
