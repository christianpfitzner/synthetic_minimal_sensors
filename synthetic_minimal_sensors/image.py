

# rosnode which publishes a simple image message every 0.1 seconds
# in the size of 128x128 pixels with some color waves

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import time


class SyntheticImage(Node):

    def __init__(self):
        super().__init__('synthetic_image')

        # create publisher for turtle_cmd_vel
        self.publisher = self.create_publisher(
            Image,
            'image',
            1)

        self.timer = self.create_timer(
            0.1,
            self.timer_callback)

        self.i = 0

    def timer_callback(self):
        # create message
        msg                 = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.height          = 128
        msg.width           = 128
        msg.encoding        = "rgb8"
        msg.is_bigendian    = False
        msg.step            = 128 * 3
        msg.data            = self.create_image()

        # publish message
        self.publisher.publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.data[0])

    def create_image(self):
        img = np.zeros((128, 128, 3), np.uint8)

        img[:, :, 0] = 128 + 128 * np.sin(self.i * np.pi / 180)
        img[:, :, 1] = 128 + 128 * np.sin(self.i * np.pi / 180 + 2 * np.pi / 3)
        img[:, :, 2] = 128 + 128 * np.sin(self.i * np.pi / 180 + 4 * np.pi / 3)

        self.i += 5

        return img.tobytes()
    
def main(args=None):
    rclpy.init(args=args)

    synthetic_image = SyntheticImage()

    rclpy.spin(synthetic_image)

    synthetic_image.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    