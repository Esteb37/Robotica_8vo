#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan    
import numpy as np

class Robot(): 
    """
    This class implements the differential drive model of the robot 
    """
    def __init__(self):
        ############ Variables ############### 
        self.x     = 0.0 # X position of the robot [m] 
        self.y     = 0.0 # Y position of the robot [m] 
        self.theta = 0.0 # Angle of the robot [rad]   

    def update_state(self, right_wheel_speed, left_wheel_speed, delta_t):
        #This function returns the robot's state 
        #This functions receives the wheel speeds right_wheel_speed and left_wheel_speed in [rad/sec]  
        #and returns the robot's state
        self.wheel_radius = 0.05
        self.robot_length = 0.19
        v = self.wheel_radius*(right_wheel_speed + left_wheel_speed)/2 
        w = self.wheel_radius*(right_wheel_speed - left_wheel_speed)/self.robot_length   
        self.theta = self.theta + w*delta_t 
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta)) 
        vx = v*np.cos(self.theta) 
        vy = v*np.sin(self.theta)
        self.x = self.x + vx*delta_t
        self.y = self.y + vy*delta_t       
 
class GoToGoal_Bug0():  
    '''
    This class will make the puzzlebot move to a given goal and implement the bug0 algorithm
    '''  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)
        ############ Variables ############### 
        self.delta_t = 1.0 / 10.0
        self.robot_length = 0.19
        self.wheel_radius = 0.05
        self.linear_speed_gain = 0.3
        self.angular_speed_gain = 1.2
        self.max_linear_speed = 0.3
        self.max_angular_speed = 0.8
        self.exponential_growth_rate_linear = 2.0
        self.exponential_growth_rate_angular = 2.0
        self.target_position_tolerance = 0.10
        self.threshold = 0.20
        self.forward_velocity = 0.1
        self.forward_angular_speed = 1.75
        self.blended_controller_distance = 0.42

        ############  Robot velocity and object ############### 
        self.robot_vel = Twist()      
        self.robot = Robot() 

        self.goal_x = 0.0 
        self.goal_y = 0.0  
        self.distance_to_goal_start = 0.0  
        self.distance_to_goal_step = 0.0  
        self.distance_to_line = 0.0
        self.right_wheel_speed = 0.0  
        self.left_wheel_speed = 0.0      
    
        self.is_lidar_received = False 
        self.is_goal_received = False
        cte = 0 ; cte1 = -1   
        self.current_state = "Stop" # Robot's current state     

        bug0_flag = True

        ###******* INIT PUBLISHERS *******###
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        
        ############ SUBSCRIBERS #############  
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_cb) 
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 
 
        #********** INIT NODE **********###
        rate = rospy.Rate(10) #freq Hz  
        rospy.loginfo("Node initialized") 

        ################################ MAIN LOOP ################################
        while not rospy.is_shutdown():  
            self.robot.update_state(self.right_wheel_speed, self.left_wheel_speed, self.delta_t) 
            if self.is_lidar_received: 
                 
                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle 
                thetaAO = self.compute_theta_ao(closest_angle) #get the angle for the avoid obstacle behavior
                self.distance_to_goal_start = np.sqrt((self.goal_x-self.robot.x)**2+(self.goal_y-self.robot.y)**2) #calculate distance at the beginning
                if self.is_goal_received: 
                    self.is_goal_received = False
                    if self.goal_x == 0.0: self.goal_x = 1e-6
                    cte = self.goal_y/self.goal_x
                self.distance_to_line = abs(cte*self.robot.x + cte1*self.robot.y)/(np.sqrt(cte**2+cte1**2))

                #******************** STATE MACHINE **********************
                if self.current_state == "GoToGoal":                  
                    if closest_range <= self.blended_controller_distance: 
                        # Implement the following walls behavior 
                        rospy.loginfo("I'm following walls now") 
                        self.distance_to_goal_step = self.distance_to_goal_start
                        self.current_state = "Clockwise"                    
                    else: 
                        v_gtg, w_gtg, thetaGTG = self.compute_gtg_control(self.goal_x, self.goal_y, self.robot.x, self.robot.y, self.robot.theta) 
                        self.robot_vel.linear.x = v_gtg 
                        self.robot_vel.angular.z = w_gtg 
 
                elif self.current_state == "Clockwise": 
                    if bug0_flag and (self.distance_to_goal_start < self.distance_to_goal_step - self.threshold and abs(thetaAO - thetaGTG) < np.pi/2):
                        self.current_state = "GoToGoal" 
                    elif (self.distance_to_goal_start <= self.distance_to_goal_step - self.threshold) and self.distance_to_line < 0.25:
                        self.current_state = "GoToGoal"  
                    else: 
                        thetaFWC = self.get_theta_fw(thetaAO, True) #Clockwise or counterclockwise 
                        vFWC, wFWC = self.compute_fw_control(thetaFWC) 
                        self.robot_vel.linear.x = vFWC 
                        self.robot_vel.angular.z = wFWC                         
                 
                elif self.at_goal() == True:
                    rospy.loginfo("Stopping movement") 
                    rospy.loginfo(self.distance_to_goal_start)
                    self.current_state = "Stop" #Change state to stop so we dont hit the object
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0
                
                elif self.current_state == "Stop":
                    rospy.loginfo("Stopping movement")
                    rospy.loginfo(self.distance_to_goal_start) 
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0
                
                elif self.at_goal() == False:
                    rospy.loginfo("Give me a goal to follow!")
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0
                         
            self.pub_cmd_vel.publish(self.robot_vel)
            rate.sleep()  
     
    def at_goal(self): 
        #This function returns true if the robot is close enough to the goal 
        at_goal = np.sqrt((self.goal_x-self.robot.x)**2+(self.goal_y-self.robot.y)**2) < self.target_position_tolerance 
        return at_goal 
 
    def get_closest_object(self, lidar_msg): 
        #This function returns the closest object to the robot 
        #This functions receives a ROS LaserScan message and returns the distance and 
        #direction to the closest object, returns  closest_range [m], closest_angle [rad]

        min_idx = np.argmin(lidar_msg.ranges) 
        closest_range = lidar_msg.ranges[min_idx] 
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 
        return closest_range, closest_angle 
 
    def compute_gtg_control(self, goal_x, goal_y, x_robot, y_robot, theta_robot): 
        ## This function returns the linear and angular speed to reach a given goal
        ed=np.sqrt((goal_x-x_robot)**2+(goal_y-y_robot)**2) 
        theta_target=np.arctan2(goal_y-y_robot,goal_x-x_robot) 
        e_theta=theta_target-theta_robot 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 
 
        self.angular_speed_gain = self.max_angular_speed*(1-np.exp(-self.exponential_growth_rate_angular*e_theta**2))/abs(e_theta) 
        w  = self.angular_speed_gain*e_theta 
        
        if abs(e_theta) > np.pi/8: 
            v=0  
        else: 
            self.linear_speed_gain=self.max_linear_speed*(1-np.exp(-self.exponential_growth_rate_linear*ed**2))/abs(ed) 
            v=self.linear_speed_gain*ed

        return v, w, e_theta
 
    def compute_theta_ao(self, theta_closest): 
        ### This function returns the angle for the Avoid obstacle behavior
        thetaAO=theta_closest-np.pi 
        thetaAO = np.arctan2(np.sin(thetaAO),np.cos(thetaAO)) 
        return thetaAO 
  
    def get_theta_fw(self, thetaAO, clockwise): 
        ### This function computes the linear and angular speeds for the robot
        if(clockwise):
            theta_fw = np.pi/2 + thetaAO
        else:
            theta_fw = - np.pi/2 + thetaAO
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))     
        return theta_fw  
     
    def compute_fw_control(self, thetaFW): 
        ##This function computes the linear and angular speeds for the robot 
        v = self.forward_velocity
        w = self.forward_angular_speed*thetaFW 
        return v,w
    
    def get_angle(self, idx, angle_min, angle_increment):  
        ## This function returns the angle for a given element of the object in the lidar's frame  
        angle= angle_min + idx * angle_increment  
        # Limit the angle to [-pi,pi]  
        angle = np.arctan2(np.sin(angle),np.cos(angle))  
        return angle
    
    # =============== Callbacks =========================
    def laser_cb(self, msg):   
        ## This function receives a message of type LaserScan   
        self.lidar_msg = msg  
        self.is_lidar_received = True  
 
    def wl_cb(self, left_wheel_speed):  
        ## This function receives a the left wheel speed [rad/s] 
        self.left_wheel_speed = left_wheel_speed.data 
         
    def wr_cb(self, right_wheel_speed):  
        ## This function receives a the right wheel speed.  
        self.right_wheel_speed = right_wheel_speed.data  
     
    def goal_cb(self, goal):  
        ## This function receives a the goal from rviz.  
        rospy.loginfo("Goal received I'm moving to x= "+str(goal.pose.position.x)+" y= "+str(goal.pose.position.y)) 
        self.current_state = "GoToGoal" 
        self.goal_x = goal.pose.position.x 
        self.goal_y = goal.pose.position.y 
        self.is_goal_received = True   # Indicate if the laser scan has been received 
 
    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 
 
############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("bug_0", anonymous=True)  
    GoToGoal_Bug0() 
