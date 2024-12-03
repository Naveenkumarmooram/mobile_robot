import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Publishers and Subscribers
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set initial robot state
        self.state = 'FOLLOW_WALL'
        self.turn_direction = 'RIGHT'  # Default to following the right wall

    def laser_callback(self, msg):
        # Laser scan distances for front, left, and right (SWAPPED LEFT AND RIGHT)
        front_distance = min(msg.ranges[170:190])  # Front (180 degrees)
        right_distance = min(msg.ranges[80:100])  # Actually the left (90 degrees)
        left_distance = min(msg.ranges[260:280])  # Actually the right (270 degrees)

        self.get_logger().info(f'Front: {front_distance}, Left: {left_distance}, Right: {right_distance}')

        # Wall-following state behavior
        if self.state == 'FOLLOW_WALL':
            self.follow_wall(left_distance, right_distance, front_distance)
        elif self.state == 'AVOID_OBSTACLE':
            self.avoid_obstacle(front_distance)

    def move_straight(self):
        move_msg = Twist()
        move_msg.linear.x = 0.1  # Move forward slowly
        move_msg.angular.z = 0.0  # No turning
        self.cmd_vel_publisher.publish(move_msg)
        self.get_logger().info('Moving straight...')

    def stop_moving(self):
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(move_msg)
        self.get_logger().info('Stopping...')

    def turn_left(self):
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.angular.z = 0.5  # Turn left
        self.cmd_vel_publisher.publish(move_msg)
        self.get_logger().info('Turning left...')

    def turn_right(self):
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.angular.z = -0.5  # Turn right
        self.cmd_vel_publisher.publish(move_msg)
        self.get_logger().info('Turning right...')

    def follow_wall(self, left_distance, right_distance, front_distance):
        # If there's an obstacle in front, avoid it
        if front_distance < 0.7:
            self.state = 'AVOID_OBSTACLE'
            self.stop_moving()
            return

        # Prioritize following the right wall if it's close
        if right_distance < 1.0:  # If the right wall is close, follow it
            self.get_logger().info('Following right wall...')
            if right_distance < 0.3:  # Too close to the right wall, move left slightly
                self.turn_left_slightly()
            elif right_distance > 0.5:  # Too far from the right wall, move right slightly
                self.turn_right_slightly()
            else:
                self.move_straight()  # Maintain distance
        # If there's no right wall, follow the left wall
        elif left_distance < 1.0:
            self.get_logger().info('Following left wall...')
            if left_distance < 0.3:  # Too close to the left wall, move right slightly
                self.turn_right_slightly()
            elif left_distance > 0.5:  # Too far from the left wall, move left slightly
                self.turn_left_slightly()
            else:
                self.move_straight()  # Maintain distance
        else:
            # If no walls nearby, move straight
            self.get_logger().info('No walls detected, moving straight...')
            self.move_straight()

    def turn_left_slightly(self):
        move_msg = Twist()
        move_msg.linear.x = 0.05
        move_msg.angular.z = 0.3  # Slightly turn left
        self.cmd_vel_publisher.publish(move_msg)
        self.get_logger().info('Turning left slightly...')

    def turn_right_slightly(self):
        move_msg = Twist()
        move_msg.linear.x = 0.05
        move_msg.angular.z = -0.3  # Slightly turn right
        self.cmd_vel_publisher.publish(move_msg)
        self.get_logger().info('Turning right slightly...')

    def avoid_obstacle(self, front_distance):
        # If there's an obstacle in front, avoid it
        self.get_logger().info('Avoiding obstacle...')
        if front_distance < 0.7:
            if self.turn_direction == 'RIGHT':
                self.turn_left()  # Avoid by turning left
            else:
                self.turn_right()  # Avoid by turning right

        # After avoiding, resume wall-following
        self.state = 'FOLLOW_WALL'

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
