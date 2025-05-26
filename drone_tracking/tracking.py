import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os


class Detection(Node):
    def __init__(self):
        super().__init__('detection')

        # Inicializa câmera
        self.cap = cv.VideoCapture(0)
        self.cap.set(cv.CAP_PROP_FPS, 30)  # Define na câmera, se ela suportar
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

        fps = self.cap.get(cv.CAP_PROP_FPS)
        self.get_logger().info(f"FPS configurado na câmera: {fps}")

        # Publisher de imagem
        self.camera_publishing = self.create_publisher(Image, '/camera', 10)
        self.bridge = CvBridge()

        # Timer para capturar e publicar a cada ~33ms (30 FPS)
        timer_period = 1.0 / 30.0  # 33ms
        self.timer = self.create_timer(timer_period, self.capture_callback)

    def capture_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Erro ao capturar frame da câmera")
            return

        # Publica imagem no tópico
        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.camera_publishing.publish(ros_image)

    def destroy_node(self):
        # Libera recursos ao encerrar
        self.cap.release()
        cv.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
