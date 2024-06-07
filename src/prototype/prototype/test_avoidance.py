import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from launch.substitutions import LaunchConfiguration
from random import randint

class Avoidance_Test(Node):

    def __init__(self):
        super().__init__('LaserTest')
        self.declare_parameter('bot_id', "null")
        self.id = self.get_parameter('bot_id').value
        #robot speed control
        self.forward_speed = 0.4
        self.turn_speed = 0.5

        #robot laser scan holders
        self.left_distance = 0
        self.right_distance = 0
        self.forward_distance = 0
        self.left_forward_distance = 0
        self.right_forward_distance = 0

        #robot obstacle threshold distance (how close to an obstacle can it get to)
        self.obstacle_threshold = 0.9
        self.obstacle_threshold_side = 0.5

        self.scan_sub = self.create_subscription(LaserScan, str(self.id)+'/scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(Twist, str(self.id)+'/cmd_vel', 10)

        self.get_logger().info("Node Active")


    def scan_callback(self, msg):
        #self.get_logger().info("Laser Scan Received") #uncomment to check laser scan messages are being received
        self.left_distance = msg.ranges[randint(289,360)]
        self.left_forward_distance = msg.ranges[randint(217,288)]
        self.forward_distance = msg.ranges[randint(145,216)]
        self.right_forward_distance = msg.ranges[randint(73,144)]
        self.right_distance = msg.ranges[randint(0,72)]

        self.obstacle_avoidance()

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

    def debug(self, print_msg):
        self.get_logger().info(self.id+' : '+str(print_msg))

def main(args=None):
    rclpy.init(args=args)
    node = Avoidance_Test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()