"""
Terminology
m = meters
m/s = meters per second
rad = radians
rad/s = radians per second
lead bot = the lead robot of the entire swarm
s = seconds
"""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from std_msgs.msg import String
import json

from random import randint
import math
from time import sleep

class Swarm_Bot_Node(Node):

    #initialise this node class
    def __init__(self):
        super().__init__('Swarm_Bot')

        sleep(5)

        #set robot id from parameter given by launcher
        self.declare_parameter('bot_id', "null")
        self.id = self.get_parameter('bot_id').value
        self.id_number = int(self.id.replace('bot',''))

        #declare if this robot is the leader
        self.leader_bot = (self.id == 'bot0')

        #pose of the leader this robot follows
        self.leader_current_pose = {'x':0,'y':0, 'yaw':0}
        
        #set formation values
        self.formation_command = 'triangle'
        self.formation_size = 2.5 #\m
        self.wing = 0
        self.yaw_to_match = 0.0 #\rad
        self.previous_formation = self.formation_command

        #true if this robot has a foller
        self.follower_found = False

        #the leader has two followers while all others only have one
        if self.leader_bot:
            self.follower_id = {'bot_name_1' : "", 'bot_name_2': ""}
        else:
            self.follower_id = ""
        

        #true if this robot has found a leader or is the lead robot
        self.leader_found = (self.leader_bot)

        #true if this robot is the last of its wing
        self.back_end = False

        #true if follower/s have reached their goal 
        if self.leader_bot:
            self.follower_goal = {}
        else:
            self.follower_goal = False

        #robot position and yaw
        self.current_pose = {'x':0,'y':0, 'yaw':0}
        self.yaw_precision = 2 * (math.pi / 180) #+/- prescision error for a desired angle change \rad
        self.full_turn = math.pi - (5 * (math.pi / 180)) #+/- prescision error for a 180 degree turn \rad
        self.full_turn_start = 0
        self.start_yaw = 0


        #values to determin how close to an obstacle the robot can be before it starts to avoid
        self.obstacle_threshold = 0.6   #\m
        self.obstacle_threshold_trace = 0.1    #how close to an obstacle the robot can be while following it \m
        self.obstacle_location = {'x':0,'y':0,'h':False}    #saves the location of when it reached an obstacle

        #the robots current mode
        self.mode = "goal"

        #values for reaching the goal
        self.goal_precision = 0.1   #how close to the goal the robot needs to get to \m
        self.goal_reached = False
        self.goal_history = []

        #robot speed control
        self.forward_speed = 0.5 #\m/s
        self.forward_speed_slow = 0.5 #\m/s
        self.forward_speed_fast = 0.6 #\m/s
        self.turn_speed = 0.4 #\rad/s
        self.turn_speed_yaw_adjustment = 1.0 #\rad/s
        self.turn_speed_slow = 0.3 #\rad/s
        self.turn_speed_fast = 0.5 #\rad/s

        #robot laser scan values
        self.left_distance = 0
        self.right_distance = 0
        self.forward_distance = 0
        self.left_forward_distance = 0
        self.right_forward_distance = 0
        

        #publish and subscriptions the robot uses
        self.vel_pub = self.create_publisher(Twist, str(self.id)+'/cmd_vel', 10) #used to send velocities to the diff drive plugin
        self.leader_pose_pub = self.create_publisher(String, str(self.id)+'/goal_pos', 10) #publisher for this robot to send goal positions to its followers

        self.scan_sub = self.create_subscription(LaserScan, str(self.id)+'/scan', self.scan_callback, qos_profile=qos_profile_sensor_data) #lidar readings
        self.odom_sub = self.create_subscription(Odometry, str(self.id)+'/odom', self.odom_callback, 10) #recieve odometry values

        self.follower_com_sub = self.create_subscription(String, str(self.id) +'/follower_com', self.follower_callback, 10) #recieve communcations from follower
        self.follower_com_pub = self.create_publisher(String, str(self.id) +'/follower_com', 10) #send communication to follower

        if(self.leader_bot):
            self.goal_pose_sub = self.create_subscription(String, 'test_pose', self.goal_callback, 10) #the lead bot recieves goal commands
            self.wing = 0
        else:
            
            #determin which wing the robot belongs to 
            if self.id_number % 2 == 0:
                self.wing = 1
            else:
                self.wing = 2

            #subscribe and publish to this leaders topics
            if self.id_number == 1:
                self.leader_pose_sub = self.create_subscription(String, 'bot0/goal_pos',self.goal_callback, 10)
                self.leader_odom_sub = self.create_subscription(Odometry, 'bot0/odom', self.leader_odom_callback, 10)
                self.leader_com_pub = self.create_publisher(String, 'bot0/follower_com', 10)
                self.leader_com_sub = self.create_subscription(String, 'bot0/follower_com', self.leader_callback, 10)
            else:
                self.leader_com_pub = self.create_publisher(String, 'bot'+ str(self.id_number-2) +'/follower_com', 10)
                self.leader_com_sub = self.create_subscription(String, 'bot'+ str(self.id_number-2) +'/follower_com', self.leader_callback, 10)
                self.leader_pose_sub = self.create_subscription(String, 'bot'+ str(self.id_number-2) +'/goal_pos',self.goal_callback, 10)
                self.leader_odom_sub = self.create_subscription(Odometry, 'bot'+ str(self.id_number-2)+'/odom', self.leader_odom_callback, 10)

            #create a timer to wait for all follower connections to be made
            timer_period = 5.0  #\s
            self.timer = self.create_timer(timer_period, self.timer_callback)

    #function called when timer reaches zero seconds
    def timer_callback(self):

        #if no follower found assume this robot is the last one in its wing
        if not self.follower_found:
            self.back_end = True
            self.debug("I'm the last bot of wing: " + str(self.wing))
            self.timer.cancel()

    #check left most laser
    def check_left(self, dist = 0.0):
        return self.left_distance < self.obstacle_threshold - dist
    
    #check left forward laser
    def check_left_front(self, dist = 0.0):
        return self.left_forward_distance < self.obstacle_threshold - dist
    
    #check front facing laser
    def check_front(self, dist = 0.0):
        return self.forward_distance < self.obstacle_threshold - dist
    
    #check right forward laser
    def check_right_front(self, dist = 0.0):
        return self.right_forward_distance < self.obstacle_threshold- dist
    
    #check right most laser
    def check_right(self, dist = 0.0):
        return self.right_distance < self.obstacle_threshold - dist

    #caluclate the euclidean distance between two points
    def euclidean_distance(self, start, goal):
        return math.sqrt(pow(goal['x'] - start['x'], 2) + pow(goal['y'] - start['y'], 2))
    
    #determin goal poses to send for a particular formation to send to this robots follower
    def formation_orientations(self, formation):
        
        #set yaw the formation should be facing
        desired_yaw = self.current_pose['yaw']
        
        #set yaw to be the yaw recieved from leader
        if not self.leader_bot:
            desired_yaw = self.yaw_to_match

        #initalise pose data to send
        pose_to_send = {
            '1':{'x': 0.0, 'y':0.0},
            '2':{'x': 0.0, 'y':0.0},
            'wing':0,
            'formation' : self.formation_command,
            'yaw' : desired_yaw
            }

        if formation == "triangle":

            pose_to_send['1']['x'] = self.current_pose['x'] - self.formation_size * math.sin(math.pi/3 + desired_yaw)
            pose_to_send['1']['y'] = self.current_pose['y'] + self.formation_size * math.cos(math.pi/3 + desired_yaw)
            pose_to_send['2']['x'] = self.current_pose['x'] - self.formation_size * math.sin(2*math.pi/3 + desired_yaw)
            pose_to_send['2']['y'] = self.current_pose['y'] + self.formation_size * math.cos(2*math.pi/3 + desired_yaw)

            if self.wing != 0:
                pose_to_send['wing'] = self.wing

        elif formation == "landscape":
            pose_to_send['1']['x'] = self.current_pose['x'] + self.formation_size * math.cos(math.pi/2 + self.current_pose['yaw'])
            pose_to_send['1']['y'] = self.current_pose['y'] + self.formation_size * math.sin(math.pi/2 + self.current_pose['yaw'])
            pose_to_send['2']['x'] = self.current_pose['x'] - self.formation_size * math.cos(math.pi/2 + self.current_pose['yaw'])
            pose_to_send['2']['y'] = self.current_pose['y'] - self.formation_size * math.sin(math.pi/2+self.current_pose['yaw'])
        
        elif formation == "vertical":
            horizontal_size = self.formation_size / 2
            pose_to_send['1']['x'] = self.current_pose['x'] - horizontal_size * math.cos(desired_yaw)
            pose_to_send['1']['y'] = self.current_pose['y'] - horizontal_size * math.sin(desired_yaw)
            pose_to_send['2']['x'] = self.current_pose['x'] - 2 * horizontal_size * math.cos(desired_yaw)
            pose_to_send['2']['y'] = self.current_pose['y'] - 2 * horizontal_size * math.sin(desired_yaw)

            if self.wing != 0:
                pose_to_send['wing'] = 2
        
        elif formation == "diamond":
            pose_to_send['1']['x'] = self.current_pose['x'] - self.formation_size * math.sin(math.pi/3 + self.current_pose['yaw'])
            pose_to_send['1']['y'] = self.current_pose['y'] + self.formation_size * math.cos(math.pi/3 + self.current_pose['yaw'])
            pose_to_send['2']['x'] = self.current_pose['x'] - self.formation_size * math.sin(2*math.pi/3 + self.current_pose['yaw'])
            pose_to_send['2']['y'] = self.current_pose['y'] + self.formation_size * math.cos(2*math.pi/3 + self.current_pose['yaw'])

            if self.id_number == 1:
                pose_to_send['wing'] = 1

        #prepare and return the pose to send
        string_msg = String()
        string_msg.data = json.dumps(pose_to_send)

        return string_msg

    #function called when a message from the leader is recieved
    def leader_callback(self, msg):
        
        #if leader responds set leader found to true
        leader_status = json.loads(msg.data)
        if leader_status['id'] != self.id:
            if not self.leader_found and leader_status['recip'] == self.id:
                self.leader_found = leader_status['heard']
                self.debug("Leader Found: "+leader_status['id'])

    #function called when a message from the follower is recieved
    def follower_callback(self, msg):

        follower_status = json.loads(msg.data)

        if follower_status['id'] != self.id:

            #lead bot waits to recieve messages from two followers to confirm they are following
            if self.leader_bot:
                if not self.follower_found:

                    if self.follower_id['bot_name_1'] == "" or self.follower_id['bot_name_1'] == follower_status['id']:
                        self.follower_id.update({'bot_name_1':follower_status['id']})
                        self.debug("Follower Found: "+follower_status['id'])
                        self.send_follower_message(follower_status['id'])
                    elif self.follower_id['bot_name_2'] == "" or self.follower_id['bot_name_2'] == follower_status['id']:
                        self.follower_id.update({'bot_name_2':follower_status['id']})
                        self.debug("Follower Found: "+follower_status['id'])
                        self.send_follower_message(follower_status['id'])
                        self.follower_found = True
                #update follower goal to contain both followers current goal status
                else:
                    self.follower_goal.update({follower_status['id'] : follower_status['goal']})
            else:
                #robot waits for a follower to send a message and confirms
                if not self.follower_found:
                    self.follower_found = follower_status['confirm']
                    self.follower_id = follower_status['id']
                    self.send_follower_message(follower_status['id'])
                    self.debug("Follower Found: "+follower_status['id'])
                #update follower goal to followers current goal status
                else:
                    self.follower_goal = follower_status['goal']

    #send follower robot a message
    def send_follower_message(self, recip = ""):
        #recip included for lead bot who communicates with two robots
        #send message to a follower to confirm their message has be recieved
        msg_to_send = {'id' : self.id, 'heard' : True, 'recip': recip}

        string_msg = String()
        string_msg.data = json.dumps(msg_to_send)

        self.follower_com_pub.publish(string_msg)

    #send this robots leader a message
    def send_leader_message(self, goal = False):

        #message used to request to follower a leader and send information on whether they have reached their goal
        msg_to_send = {'id' : self.id, 'goal' : goal, 'confirm' : True, 'recip': ""}

        string_msg = String()
        string_msg.data = json.dumps(msg_to_send)

        self.leader_com_pub.publish(string_msg)

    #function is called when the leader sends odometry information
    def leader_odom_callback(self, msg):

        bot_position = msg.pose.pose.position
        bot_orientation = msg.pose.pose.orientation

        #calcualte euler yaw from quaternion orientation values
        yaw = self.quaternion_to_euler(
            bot_orientation.x,
            bot_orientation.y,
            bot_orientation.z,
            bot_orientation.w
        )

        #save the leaders current pose
        self.leader_current_pose['x'] = bot_position.x
        self.leader_current_pose['y'] = bot_position.y
        self.leader_current_pose['yaw'] = yaw

    #set current position and yaw value to current_pose
    def odom_callback(self, msg):

        if not self.leader_bot and not self.leader_found:
            self.send_leader_message()

            
        bot_orientation = msg.pose.pose.orientation
        bot_position = msg.pose.pose.position

        #change quaternion values to euler yaw for adjustment calculations
        yaw = self.quaternion_to_euler(
            bot_orientation.x,
            bot_orientation.y,
            bot_orientation.z,
            bot_orientation.w
        )

        #set position and yaw value
        self.current_pose['x'] = bot_position.x
        self.current_pose['y'] = bot_position.y
        self.current_pose['yaw'] = yaw

        #lead bot does not recieve constant goal positions so all goal movement is called every odom callback
        if self.leader_bot:
            if len(self.goal_history) != 0:
                self.go_to_goal(self.goal_history[0])

        #if this robot is the last of its wing publish formation poses for a follower
        if not self.back_end:
            form_pose = self.formation_orientations(self.formation_command)
            self.leader_pose_pub.publish(form_pose)
    
    #pick a laser to use for object avoidance calculations
    def scan_callback(self, msg):

        #pick a laser to use for each distance reading
        self.left_distance = msg.ranges[180]
        self.left_forward_distance = msg.ranges[135]
        self.forward_distance = msg.ranges[90]
        self.right_forward_distance = msg.ranges[45]
        self.right_distance = msg.ranges[0]

    #load and calculate angle error
    def goal_callback(self, msg):

        goal = json.loads(msg.data)
        goal_pos = goal[str(self.wing)]

        if not self.leader_bot:
            if goal['wing'] != 0:
                self.wing = goal['wing']
            self.formation_command = goal['formation'] #change formation to leaders command
            self.yaw_to_match = goal['yaw'] #change yaw to the leaders desired yaw
            goal_pos = goal[str(self.wing)] #if wing changed needed, change goal position
        else:
            goal_history_length = len(self.goal_history)
            if  goal_history_length != 0:
                #check new goal isn't the same as the one previous and add it
                if self.goal_history[goal_history_length - 1] != goal_pos:
                    self.debug("Goal Added: " + str(goal_pos))
                    self.goal_history.append(goal_pos)
            else:
                self.goal_history.append(goal_pos) #if no goals add this goal
        
        #if this robot has a confirmed leader and isn't the lead bot start to move to the goal
        if self.leader_found and not self.leader_bot:
            self.go_to_goal(goal_pos)
    
    #go towards a given goal
    def go_to_goal(self, goal):
        
        #twist message to send to cmd_vel
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        if self.leader_bot:
            self.formation_command = goal["formation"]
            self.previous_formation = self.formation_command

        #calculate current distance from the goal
        position_error = self.euclidean_distance(self.current_pose, goal)

        #is this robot close enough to the goal yet
        self.goal_reached = position_error < self.goal_precision
            
        #check if the robot is going to encounter an obstacle and isn't alreading avoiding it
        if (self.check_front() or self.check_left_front() or self.check_right_front()) and not self.goal_reached:
            self.mode = "avoid" #change mode

            #check if an obstacle location has already been saved | if not save this location
            if not self.obstacle_location['h']:
                self.obstacle_location['x'] = self.current_pose['x']
                self.obstacle_location['y'] = self.current_pose['y']
                self.obstacle_location['h'] = True

        if self.mode == "goal":

            if not self.goal_reached:
                
                #calculate the angle error and velocity needed to turn towards goal
                destination_yaw = self.angle_error(goal)
                turn_vel = 0.0
                full_turn_required = math.fabs(destination_yaw) > self.full_turn

                if full_turn_required:
                    turn_vel = self.angle_turn_full()
                else:
                    turn_vel = self.face_angle_vel(self.angle_error_yaw(destination_yaw)) * self.turn_speed_yaw_adjustment

                #turn_vel = self.angle_turn_test(destination_yaw)
                
                
                #if a full 180 degree turn is needed stop the robot and allow it to turn
                if full_turn_required and turn_vel != 0.0:
                    twist_msg.linear.x = 0.0 #set forward movement
                else:
                    if self.leader_bot:
                        twist_msg.linear.x = self.forward_speed #set forward movement
                    else:
                        twist_msg.linear.x = self.forward_speed_fast

                #twist_msg.linear.x = self.forward_speed
                twist_msg.angular.z = turn_vel #set turning velocity

            #if goal is reached and this robot is not the lead bot | once goal is reached face the match the yaw the leader is facing
            elif not self.leader_bot:
                face_leader_angle_vel = self.face_angle_vel(self.angle_error_yaw(self.leader_current_pose['yaw']))
                if math.fabs(face_leader_angle_vel) > 0.0:
                    twist_msg.angular.z = face_leader_angle_vel

                #if follower has reached its goal OR this robot is the last of the wing, send leader a message saying its reached its goal
                elif self.follower_goal or self.back_end:
                    self.send_leader_message(self.goal_reached)

            #if goal is reached and this robot is the lead bot | remove this goal from its history
            else:
                self.goal_history.pop(0)
                self.formation_command = self.previous_formation #return to previous formation if it was changed during movement

        elif self.mode == "avoid":
            #get twist message from trace_obstacle function
            trace_twist = self.trace_obstacle()
            twist_msg.linear = trace_twist.linear
            twist_msg.angular = trace_twist.angular
            if self.leader_bot:
                self.formation_command = "horizontal"
            #if the robot is closer to the goal then it was when it ecountered the obstacle, contiune going to the goal
            if position_error < self.euclidean_distance(self.obstacle_location, goal):
                self.mode = "goal"
                self.obstacle_location['h'] = False

        #send the final twist message to cmd_vel
        self.vel_pub.publish(twist_msg)
    
    #function to generate a twist message to trace an obstacle
    def trace_obstacle(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        #reached obstacle corner - turn to follow corner
        if not self.check_left_front() and not self.check_front() and not self.check_right_front():
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = -self.turn_speed
        
        #obstacle directly infront - turn to trace right side
        elif not self.check_left_front() and self.check_front() and not self.check_right_front():
            twist_msg.angular.z = self.turn_speed_fast
        #obstacle on right side - trace
        elif not self.check_left_front() and not self.check_front() and self.check_right_front():
            #right wall too close turn to prevent crash
            if self.check_right_front(self.obstacle_threshold_trace):
                twist_msg.linear.x = self.forward_speed
                twist_msg.angular.z = self.turn_speed_fast
            #obstacle on right side go forward to trace
            else:
                twist_msg.linear.x = self.forward_speed

        #Obstacle on left side - find obstacle that was being traced on right side
        elif self.check_left_front() and not self.check_front() and not self.check_right_front():
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = -self.turn_speed_slow
        #obstacle directly infront and to the right turn away to begin tracing
        elif not self.check_left_front() and  self.check_front() and self.check_right_front():
            twist_msg.angular.z = self.turn_speed_fast

        elif self.check_left_front() and self.check_front() and not self.check_right_front():
            twist_msg.angular.z = self.turn_speed_fast

        #stuck in corner - turn to continue right side trace  
        elif self.check_left_front() and self.check_front() and self.check_right_front():
            twist_msg.angular.z = self.turn_speed_fast
            
        #narrow path detection - move forward or turn to obstacle on right
        elif self.check_left_front() and not self.check_front() and self.check_right_front():
            if self.check_left and self.check_right:
                twist_msg.linear.x = self.forward_speed
            else:
                twist_msg.linear.x = self.forward_speed
                twist_msg.angular.z = -self.turn_speed_slow

        return twist_msg
    
    #Quaternion to Euler calculation
    def quaternion_to_euler(self, x, y, z, w):

        sin_y = 2.0 * (w * z + x * y)
        cos_y = 1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(sin_y, cos_y)

        return yaw_z
    
    #calculate angle error between current yaw and goal position
    def angle_error(self, goal):

        destination_yaw = math.atan2(
            goal['y'] - self.current_pose['y'],
            goal['x'] - self.current_pose['x']
        )

        if math.fabs(destination_yaw) > self.full_turn:
            if self.full_turn_start == 0:
                self.full_turn_start += 1
            destination_yaw = math.pi
            if(self.leader_bot):
                if self.formation_command != 'horizontal':
                    self.previous_formation = self.formation_command
                self.formation_command = 'horizontal'
        return destination_yaw
    
    def angle_turn_full(self):

        if self.full_turn_start == 1:
            self.start_yaw = self.current_pose['yaw']
            self.full_turn_start += 1

        if math.fabs(self.current_pose['yaw']) < math.pi - self.yaw_precision:
            if (math.pi) < self.start_yaw + math.pi:
                return - self.turn_speed_yaw_adjustment
            else:
                return self.turn_speed_yaw_adjustment
        self.full_turn_start = 0
        self.start_yaw = 0
        return 0.0
            
    def angle_error_yaw(self, yaw):
        return yaw - self.current_pose['yaw']
    
    #calculate angular velocity to face a provided goal
    def face_angle_vel(self, error):

        #check if angle error is larger than desired precision
        if math.fabs(error) > self.yaw_precision:
            return error
        else:
            return 0.0
    
    #avoid obstacles using lidar readings (OUTDATED)
    def obstacle_avoidance(self):

        #initalise twist message and values for linear and angular velocities
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        if self.forward_distance > self.obstacle_threshold and self.left_forward_distance > self.obstacle_threshold and self.right_forward_distance > self.obstacle_threshold:
            twist_msg.linear.x = self.forward_speed
            self.debug("Moving Forward")
        elif self.left_forward_distance > self.obstacle_threshold and self.right_forward_distance > self.obstacle_threshold and self.forward_distance < self.obstacle_threshold:
            twist_msg.angular.z = self.turn_speed
            self.debug("Forward Blocked/turning left")
        elif self.left_forward_distance > self.obstacle_threshold and self.right_forward_distance < self.obstacle_threshold and self.forward_distance > self.obstacle_threshold:
            twist_msg.angular.z = self.turn_speed #left turn
            self.debug("Right Blocked/turning left")
        elif self.left_forward_distance < self.obstacle_threshold and self.right_forward_distance > self.obstacle_threshold and self.forward_distance > self.obstacle_threshold:
            twist_msg.angular.z = -self.turn_speed #right turn
            self.debug("Left Blocked/turning right")
        elif self.left_forward_distance > self.obstacle_threshold and self.right_forward_distance < self.obstacle_threshold and self.forward_distance < self.obstacle_threshold:
            twist_msg.angular.z = self.turn_speed
            self.debug("Right and Forward Blocked/turning left")
        elif self.left_forward_distance < self.obstacle_threshold and self.right_forward_distance > self.obstacle_threshold and self.forward_distance < self.obstacle_threshold:
            twist_msg.angular.z = -self.turn_speed
            self.debug("Left and Forward Blocked/turning right")
        elif self.left_forward_distance < self.obstacle_threshold and self.right_forward_distance < self.obstacle_threshold and self.forward_distance > self.obstacle_threshold:
            twist_msg.linear.x = self.forward_speed
            self.debug("Left and Right Blocked/moving forward")
        elif self.left_forward_distance < self.obstacle_threshold and self.right_forward_distance < self.obstacle_threshold and self.forward_distance < self.obstacle_threshold:
            twist_msg.angular.z = self.turn_speed
            twist_msg.linear.x = -self.forward_speed
            self.debug("All angles blocked/turning left")
        if self.left_distance < self.obstacle_threshold_side and self.right_distance > self.obstacle_threshold_side:
            twist_msg.angular.z = -self.turn_speed
        elif self.left_distance > self.obstacle_threshold_side and self.right_distance < self.obstacle_threshold_side:
            twist_msg.angular.z = self.turn_speed
        self.vel_pub.publish(twist_msg)

    #print information to the console with this robots id attached
    def debug(self, print_msg):
        if self.id_number == 0:
            self.get_logger().info(self.id+' : '+str(print_msg))

def main(args=None):
    rclpy.init(args=args)
    node = Swarm_Bot_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()