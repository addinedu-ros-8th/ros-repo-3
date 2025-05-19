#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Picamera2 초기화
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (640, 480)}
        )
        self.picam2.configure(config)
        try:
            self.picam2.start()
        except Exception as e:
            self.get_logger().error(f"Failed to start Picamera2: {e}")
            rclpy.shutdown()
            return

        # 10Hz 퍼블리시
        self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        try:
            frame = self.picam2.capture_array()
        except Exception as e:
            self.get_logger().error(f"Failed to capture frame: {e}")
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.picam2.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()