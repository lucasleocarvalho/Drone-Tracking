import rclpy
from rclpy.node import Node
import cv2 as cv
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class CameraShow(Node):
    
    def __init__(self):
        super().__init__('detection')
        cv.namedWindow("Detecção YOLO", cv.WINDOW_NORMAL)
        self.bridge = CvBridge()

        #Criação de publishers e subscribers
        self.camera_subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv.imshow('Detecção YOLO', cv_image)
        if cv.waitKey(1) == ord('q'):
            self.get_logger().info('Fechando janela')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CameraShow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()