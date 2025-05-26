import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np

class CameraShow(Node):
    
    def __init__(self):
        super().__init__('camera_show')
        cv.namedWindow("Detecção YOLO", cv.WINDOW_NORMAL)
        self.bridge = CvBridge()

        # Subscriber para imagem comprimida
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/camera/compressed',
            self.image_callback,
            10)
    
    def image_callback(self, msg):
        # Converte imagem comprimida para cv2
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv.imdecode(np_arr, cv.IMREAD_COLOR)

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
