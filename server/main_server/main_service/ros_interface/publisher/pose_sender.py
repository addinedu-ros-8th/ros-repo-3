import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped

class MapPoseForwarder(Node):
    def __init__(self):
        super().__init__('map_pose_forwarder')
        qos_pose = QoSProfile(depth=10)
        qos_pose.reliability = ReliabilityPolicy.RELIABLE

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/main_server/roscar_pose',
            qos_pose
        )
        self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.pose_callback,
            qos_pose
        )

        self.get_logger().info(
            'Forwarding /tracked_pose ➔ /main_server/roscar_pose'
        )

    def pose_callback(self, msg: PoseStamped):
        self.pose_pub.publish(msg)

def main():
    rclpy.init()
    node = MapPoseForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
