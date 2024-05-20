#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan #Lidar
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion


#This class will make the puzzlebot move to a given goal
class AutonomousNav():
	def __init__(self):
		rospy.on_shutdown(self.cleanup)

		############ Variables ###############
		self.x_target = rospy.get_param('/bug0/goal_x', 0) #x position of the goal
		self.y_target = rospy.get_param('/bug0/goal_y', 0) #y position of the goal

		self.pose_x = 0.0
		self.pose_y = 0.0
		self.pose_theta = 0.0

		self.goal_received = False #flag to indicate if the goal has been received
		self.lidar_received = 0 #flag to indicate if the laser scan has been received
		self.closest_angle = 0.0 #Angle to the closest object
		self.closest_range = np.inf #Distance to the closest object
		self.target_position_tolerance=0.2 #acceptable distance to the goal to declare therobot has arrived to it [m]
		ao_distance = 0.9 # distance from closest obstacle to activate the avoid obstacle behavior [m]
		stop_distance = 0.15 # distance from closest obstacle to stop the robot [m]
		eps=0.25 #Fat guard epsilon
		v_msg=Twist() #Robot's desired speed
		v_msg.linear.x = 0.2
		v_msg.angular.z = 0.0
		self.wr=0.0 #right wheel speed [rad/s]
		self.wl=0.0 #left wheel speed [rad/s]
		self.e_theta = 0.0
		self.theta_target = 0.0
		self.theta_AO = 0.0
		self.current_state = 'Stop' #Robot's current state

		###******* INIT PUBLISHERS *******###
		self.pub_cmd_vel = rospy.Publisher('puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=1)

		rospy.Subscriber("puzzlebot_1/scan", LaserScan, self.laser_cb)
		rospy.Subscriber("/run", Bool, self.run_cb)
		rospy.Subscriber("/odom", Odometry, self.odom_cb)

		#********** INIT NODE **********###
		freq=50
		rate = rospy.Rate(freq) #freq Hz
		rate.sleep()

		self.odom_received = False

		while not rospy.is_shutdown() and not self.odom_received:
			rate.sleep()


		################ MAIN LOOP ################
		while not rospy.is_shutdown():

			if self.lidar_received:
				closest_range, closest_angle = self.get_closest_object(
					self.lidar_msg)  # get the closest object range and angle

				if self.current_state == 'Stop':
					if self.goal_received:
						print("Going to goal")
						self.current_state = "GoToGoal"
					v_msg.linear.x = 0
					v_msg.angular.z = 0

				elif self.current_state == 'GoToGoal':

					if self.at_goal():
						print("At goal")
						self.current_state = "Stop"
						self.goal_received = 0

					elif closest_range < stop_distance:
						print("Too close")
						self.current_state = "Stop"

					elif closest_range < ao_distance:
						print("Avoid obstacle")
						self.current_state = "AvoidObstacle"
					else:
						v_gtg, w_gtg = self.compute_gtg_control(
							self.x_target, self.y_target, self.pose_x, self.pose_y, self.pose_theta)
						v_msg.linear.x = v_gtg
						v_msg.angular.z = w_gtg

				elif self.current_state == 'AvoidObstacle':
					print(closest_range)
					if closest_range > ao_distance:
						self.current_state = "GoToGoal"
						print("Going to goal")
					if self.at_goal():
						print("At goal")
						self.current_state = "Stop"
						self.goal_received = 0

					elif closest_range < stop_distance:
						print("Too close")
						self.current_state = "Stop"
					else:
						v_ao, w_ao = self.compute_ao_control(closest_angle)
						v_msg.linear.x = v_ao
						v_msg.angular.z = w_ao

			self.pub_cmd_vel.publish(v_msg)
			rate.sleep()

	def at_goal(self):
		return np.sqrt((self.x_target-self.pose_x)**2+(self.y_target-self.pose_y)**2)<self.target_position_tolerance


	def get_closest_object(self, lidar_msg):
		# This function returns the closest object to the robot
		# This functions receives a ROS LaserScan message and returns the distance and direction to the closest object
		# returns  closest_range [m], closest_angle [rad],
		new_angle_min = -np.pi/2.0
		ranges_size = len(lidar_msg.ranges)
		cropped_ranges = lidar_msg.ranges[int(ranges_size/4):int(3*ranges_size/4)]
		min_idx = np.argmin(cropped_ranges)
		closest_range = cropped_ranges[min_idx]
		closest_angle = new_angle_min + min_idx * lidar_msg.angle_increment
		# limit the angle to [-pi, pi]
		closest_angle = np.arctan2(
			np.sin(closest_angle), np.cos(closest_angle))
		return closest_range, closest_angle



	def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot):
		# This function returns the linear and angular speed to reach a given goal
		# This functions receives the goal's position (x_target, y_target) [m]
		#  and robot's position (x_robot, y_robot, theta_robot) [m, rad]
		# This functions returns the robot's speed (v, w) [m/s] and [rad/s]
		kvmax = 0.5  # linear speed maximum gain
		kwmax = 1.0  # angular angular speed maximum gain
		# kw=0.5
		av = 2.0  # Constant to adjust the exponential's growth rate
		aw = 2.0  # Constant to adjust the exponential's growth rate
		ed = np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2)
		# Compute angle to the target position
		theta_target = np.arctan2(y_target-y_robot, x_target-x_robot)
		e_theta = theta_target-theta_robot
		# limit e_theta from -pi to pi
		# This part is very important to avoid abrupt changes when error switches between 0 and +-2pi
		e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
		# Compute the robot's angular speed
		kw = kwmax*(1-np.exp(-aw*e_theta**2)) / abs(e_theta)  # Constant to change the speed
		w = kw*e_theta
		if abs(e_theta) > np.pi/8:
			# we first turn to the goal
			v = 0  # linear speed
		else:
			# Make the linear speed gain proportional to the distance to the target position
			# Constant to change the speed
			kv = kvmax*(1-np.exp(-av*ed**2))/abs(ed)
			v = kv*ed  # linear speed
		return v, w


	def compute_ao_control(self, closest_angle):
		self.theta_AO = closest_angle + np.pi/2
		#Limit the angle from -pi to pi
		self.theta_AO = np.arctan2(np.sin(self.theta_AO), np.cos(self.theta_AO))
		v = 0.1 # [m/s] Robot's linear velocity while avoiding obstacles
		kAO = 2.8 # Propotional constant for the angular speed controller
		w = kAO * self.theta_AO
		return v, w

	def progress(self):
		dx = self.x_target - self.pose_x
		dy = self.y_target - self.pose_y
		distance = np.sqrt(dx**2 + dy**2)
		return distance

	def laser_cb(self, msg):
		self.lidar_msg = msg
		self.lidar_received = 1



	def run_cb(self, msg):
		self.goal_received = msg.data

	def cleanup(self):
		#This function is called just before finishing the node
		# You can use it to clean things up before leaving
		# Example: stop the robot before finishing a node.
		vel_msg = Twist()
		self.pub_cmd_vel.publish(vel_msg)

	def odom_cb(self, msg):
		self.pose_x = msg.pose.pose.position.x
		self.pose_y = msg.pose.pose.position.y
		quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		#Convert the orientation to Euler angles
		_, _, yaw = euler_from_quaternion(quat)
		self.odom_received = True
		self.pose_theta = yaw

############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
	rospy.init_node("go_to_goal_with_obstacles1", anonymous=True)
	AutonomousNav()
