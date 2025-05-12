# main_test.py

import rclpy
from sensor_listener_node import SensorListener

def main():
    rclpy.init()
    node = SensorListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
