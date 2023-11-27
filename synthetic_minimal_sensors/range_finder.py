

# a ros node which publishes a single range finder message every 0.1 seconds

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range

import numpy as np


class SyntheticRangeFinder(Node):
    
    def __init__(self):
        super().__init__('synthetic_range_finder')

        # create publisher for turtle_cmd_vel
        self.publisher = self.create_publisher(
            Range,
            'range',
            1)

        self.timer = self.create_timer(
            0.1,
            self.timer_callback)

        self.i = 0

    def timer_callback(self):
        # create message
        msg                 = Range()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.radiation_type  = Range.INFRARED
        msg.field_of_view   = 0.1
        msg.min_range       = 0.0
        msg.max_range       = 100.0
        msg.range           = 10.0 + np.random.normal(0, 0.1)

        # publish message
        self.publisher.publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.range)

    
def main(args=None):
    rclpy.init(args=args)

    synthetic_range_finder = SyntheticRangeFinder()

    rclpy.spin(synthetic_range_finder)

    synthetic_range_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    