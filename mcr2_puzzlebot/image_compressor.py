#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class YoloImageCompressor(Node):

    def __init__(self):
        super().__init__('image_compressor')

        self.declare_parameter('input_topic', '/video_source/raw')
        self.declare_parameter('output_topic', '/video_source/yolo/compressed')
        self.declare_parameter('width',416)
        self.declare_parameter('height',416)
        self.declare_parameter('jpeg_quality',40)
        self.declare_parameter('grayscale',True)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.grayscale = self.get_parameter('grayscale').value

        self.sub = self.create_subscription(Image,input_topic,self.image_callback,10)
        self.pub = self.create_publisher(CompressedImage,output_topic,qos)
        self.get_logger().info('Yolo Image Compressor Started')

    def image_callback(self, msg):
        try:
            frame = np.frombuffer(msg.data,dtype=np.uint8).reshape((msg.height, msg.width, 3))
            frame = cv2.resize(frame,(self.width, self.height))
            if self.grayscale:
                gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                frame = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY),self.jpeg_quality]
            success, encoded_img = cv2.imencode('.jpg',frame,encode_param)

            if not success:
                return

            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded_img.tobytes()
            self.pub.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(
                f'Compression failed: {e}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = YoloImageCompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()