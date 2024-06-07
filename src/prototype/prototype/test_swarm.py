import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from time import sleep
from std_msgs.msg import String
import json
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from random import randint

class Swarm_Node(Node):
    
    def __init__(self):
        super().__init__('Swarm_Node')
        self.declare_parameter('bot_id', "null")
        self.id = self.get_parameter('bot_id').value

        self.current_pose = {'x':0,'y':0, 'yaw':0}

        self.yaw_precision = 2.0 * (math.pi / 180)

        self.obstacle_threshold = 1.0
        self.mode = "goal"
        self.obstacle_location = {'x':0,'y':0,'h':False}

        self.goal_precision = 0.5
        self.goal_reached = False

        self.left_distance = 0
        self.right_distance = 0
        self.forward_distance = 0
        self.left_forward_distance = 0
        self.right_forward_distance = 0

        self.turn_speed = 0.5
        self.turn_speed_yaw_adjustment = 0.1

        self.vel_pub = self.create_publisher(Twist, str(self.id)+'/cmd_vel', 10)
        self.goal_pose_sub = self.create_subscription(String, 'test_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, str(self.id)+'/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, str(self.id)+'/scan', self.scan_callback, qos_profile=qos_profile_sensor_data) #lidar readings

        self.debug("Node Active")

    def goal_callback(self, msg):
        goal = json.loads(msg.data)
        error = self.angle_error(goal)
        self.go_to_goal(error, goal)

    def odom_callback(self, msg):
        
        bot_orientation = msg.pose.pose.orientation
        bot_position = msg.pose.pose.position

        yaw = self.quaternion_to_euler(
            bot_orientation.x,
            bot_orientation.y,
            bot_orientation.z,
            bot_orientation.w
        )

        self.current_pose['x'] = bot_position.x
        self.current_pose['y'] = bot_position.y
        self.current_pose['yaw'] = yaw
    
    def scan_callback(self, msg):
        #self.get_logger().info("Laser Scan Received") #uncomment to check laser scan messages are being received
        self.left_distance = msg.ranges[360]
        self.left_forward_distance = msg.ranges[270]
        self.forward_distance = msg.ranges[180]
        self.right_forward_distance = msg.ranges[90]
        self.right_distance = msg.ranges[0]

    def check_left(self, dist = 0.5):
        return self.left_distance < dist
    
    def check_left_front(self, dist = 0.5):
        return self.left_forward_distance < dist
    
    def check_front(self, dist = 0.5):
        return self.forward_distance < dist
    
    def check_right_front(self, dist = 0.5):
        return self.right_forward_distance < dist
    
    def check_right(self, dist = 0.5):
        return self.right_distance < dist

    def euclidean_distance(self, start, goal):
        return math.sqrt(pow(goal['x'] - start['x'], 2) + pow(goal['y'] - start['y'], 2))
    
    def quaternion_to_euler(self, x, y, z, w):

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z
    
    def angle_error(self, goal):

        destination_yaw = math.atan2(
            goal['y'] - self.current_pose['y'],
            goal['x'] - self.current_pose['x']
        )

        error = destination_yaw - self.current_pose['yaw']
        #self.debug(destination_yaw)
        #self.debug(self.current_pose['yaw'])
        return error
    
    def face_angle(self, error):

        #initalise twist message
        #twist_msg = Twist()
        #twist_msg.angular.z = 0.0

        #check if angle error is larger than desired precision
        if math.fabs(error) > self.yaw_precision:
            if(error > 0):
                #left turn to face goal
                return self.turn_speed * math.fabs(error) + self.turn_speed_yaw_adjustment
                self.debug("Turning left")
            else:
                #right turn to face goal
                return -self.turn_speed * math.fabs(error) + self.turn_speed_yaw_adjustment
                self.debug("Turning right")
        else:
            return 0.0

        self.vel_pub.publish(twist_msg)

    def trace_obstacle(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        if not self.check_left_front() and not self.check_front() and not self.check_right_front():
            twist_msg.linear.x = self.turn_speed
            twist_msg.angular.z = -self.turn_speed
             
        elif not self.check_left_front() and self.check_front() and not self.check_right_front():
            twist_msg.angular.z = self.turn_speed

        elif not self.check_left_front() and not self.check_front() and self.check_right_front():
            if self.check_right_front(self.obstacle_threshold/1.75):
                twist_msg.linear.x = self.turn_speed
                twist_msg.angular.z = self.turn_speed
            else:
                twist_msg.linear.x = self.turn_speed

        elif self.check_left_front() and not self.check_front() and not self.check_right_front():
            twist_msg.linear.x = self.turn_speed
            twist_msg.angular.z = -self.turn_speed
             
        elif not self.check_left_front() and  self.check_front() and self.check_right_front():
            twist_msg.angular.z = self.turn_speed
             
        elif self.check_left_front() and self.check_front() and not self.check_right_front():
            twist_msg.angular.z = self.turn_speed
             
        elif self.check_left_front() and self.check_front() and self.check_right_front():
            twist_msg.angular.z = self.turn_speed
             
        elif self.check_left_front() and not self.check_front() and self.check_right_front():
            twist_msg.linear.x = self.turn_speed
            twist_msg.angular.z = -self.turn_speed

        return twist_msg
    def go_to_goal(self, error, goal):
        
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        
        if self.check_front() or self.check_left_front() or self.check_right_front():
            self.mode = "avoid"

            if not self.obstacle_location['h']:
                self.obstacle_location['x'] = self.current_pose['x']
                self.obstacle_location['y'] = self.current_pose['y']
                self.obstacle_location['h'] = True

        if self.mode == "goal":

            position_error = self.euclidean_distance(self.current_pose, goal)
            if position_error > self.goal_precision:

                twist_msg.angular.z = self.face_angle(error)
                twist_msg.linear.x = self.turn_speed

        elif self.mode == "avoid":

            trace_twist = self.trace_obstacle()
            twist_msg.linear = trace_twist.linear
            twist_msg.angular = trace_twist.angular

            if self.euclidean_distance(self.current_pose, goal) < self.euclidean_distance(self.obstacle_location, goal):
                self.mode = "goal"
                self.obstacle_location['h'] = False

        self.vel_pub.publish(twist_msg)

    def debug(self, print_msg):
        self.get_logger().info(self.id+' : '+str(print_msg))

def main(args=None):
    rclpy.init(args=args)
    node = Swarm_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()