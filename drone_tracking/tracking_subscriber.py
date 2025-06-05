import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2 as cv
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
from ament_index_python import get_package_share_directory
import os


class CameraShow(Node):
    
    def __init__(self):
        super().__init__('camera_show')
        cv.namedWindow("Detecção YOLO", cv.WINDOW_NORMAL)
        self.bridge = CvBridge()

        #Definição de QoS pra reduzir delay na transmissão da câmera
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        #Definição de variáveis
        self.frame_c = 0
        self.annoted_frame = None

        # Subscriber para imagem comprimida
        self.camera_subscription = self.create_subscription(CompressedImage, '/camera/compressed', self.image_callback, qos)
    
    def image_callback(self, msg):
        # Converte imagem comprimida para cv2
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)

        #Import da rede neural
        package_dir = get_package_share_directory('drone_tracking')
        model_path = os.path.join(package_dir, 'net_train', 'weights', 'best.pt')
        self.model = YOLO(model_path)

        #Detecção com YOLO
        self.frame_c += 1
        if self.frame_c % 3 == 0:
            results = self.model(frame, verbose=False, conf=0.5)
            self.annoted_frame = np.array(results[0].plot())
        else:
            self.annoted_frame = None

        #Frame para exibição
        frame_show = self.annoted_frame if self.annoted_frame is not None else frame

        #Envio dos dados para o algoritmo de EKF
        self.ekf(self.annoted_frame)

        cv.imshow('Detecção YOLO', frame_show)
        if cv.waitKey(1) == ord('q'):
            self.get_logger().info('Fechando janela')
            rclpy.shutdown()

    def ekf(self, frame):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraShow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
