import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import os
import time

class CircleMotion(Node):
    def __init__(self):
        super().__init__('circle_motion')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_in_circle)
        self.angular_speed = 1.8  # radians per second
        self.linear_speed = 0.2   # meters per second

    def move_in_circle(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher.publish(msg)
        self.get_logger().info('Moving in a circle')

def main(args=None):
    rclpy.init(args=args)
     # Wait for the robot to spawn
    while os.environ.get('ROBOT_SPAWNED', 'false').lower() != 'true':
        #print("Waiting for robot to spawn...")
        time.sleep(1)
    
    print("Robot spawned. Starting circle motion.")
    circle_motion = CircleMotion()
    rclpy.spin(circle_motion)
    
    circle_motion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()