

# This example creates a minimal example of lidar data which is published via ros every 0.1 seconds.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import numpy as np
import math
import time


class SyntheticLidar(Node):

    def __init__(self):
        super().__init__('synthetic_lidar')

        # create publisher for turtle_cmd_vel
        self.publisher = self.create_publisher(
            LaserScan,
            'scan',
            1)

        self.timer = self.create_timer(
            0.1,
            self.timer_callback)

        self.i = 0

    def timer_callback(self):
        # create message
        msg                 = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.angle_min       = -math.pi / 2
        msg.angle_max       = math.pi / 2
        msg.angle_increment = math.pi / 180
        msg.time_increment  = 0.0001
        msg.scan_time       = 0.0001
        msg.range_min       = 0.0
        msg.range_max       = 100.0

        for i in range(0, 180):
            noise = np.random.normal(0, 0.1)
            msg.ranges.append( 10 * math.sin(i * math.pi / 180) + noise)
            msg.intensities.append(100)



        # publish message
        self.publisher.publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.ranges[0])




def main(args=None):
    rclpy.init(args=args)

    synthetic_lidar = SyntheticLidar()

    rclpy.spin(synthetic_lidar)

    synthetic_lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





