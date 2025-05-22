import rclpy
from rclpy.node import Node
from shared_interfaces.msg import NavigationGoal

class NavigationGoalPublisher(Node):
    def __init__(self):
        super().__init__('navigation_goal_publisher')
        self.pub = self.create_publisher(NavigationGoal, '/roscar/navigation/goal', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)

    def publish_goal(self):
        msg = NavigationGoal()
        msg.roscar_id = 1
        msg.goal_x = 3.5
        msg.goal_y = -1.2
        msg.theta = 1.57
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.get_logger().info(f'발행: NavigationGoal {msg}')

def main():
    rclpy.init()
    node = NavigationGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
