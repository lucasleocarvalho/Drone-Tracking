import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2 as cv
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import os
from ultralytics import YOLO
from ament_index_python import get_package_share_directory
import numpy as np


class Detection(Node):
    def __init__(self):
        super().__init__('detection')

        #Definição de QoS pra reduzir delay na transmissão da câmera
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Inicializa câmera
        self.cap = cv.VideoCapture(0, cv.CAP_V4L2)
        self.cap.set(cv.CAP_PROP_FPS, 30)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv.CAP_PROP_BUFFERSIZE, 1)


        fps = self.cap.get(cv.CAP_PROP_FPS)
        self.get_logger().info(f"FPS configurado na câmera: {fps}")

        #Publisher de imagem
        self.camera_publishing = self.create_publisher(Image, '/camera/image_raw', qos)
        self.bridge = CvBridge()

        #Variáveis de controle
        self.frame_c = 0
        self.annoted_frame = None

        #Import da rede neural
        package_dir = get_package_share_directory('drone_tracking')
       # model_path = os.path.join(package_dir, 'net_train', 'weights', 'best.pt')
       # self.model = YOLO(model_path)
       # self.model.export(format="ncnn")
        self.model = YOLO(os.path.join(package_dir, 'net_train', 'weights', 'best_ncnn_model'))

        #Timer para 30 FPS
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.capture_callback)

    def capture_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Erro ao capturar frame da câmera")
            return


        #Detecção com YOLO
        self.frame_c += 1
        if self.frame_c % 10 == 0:
            results = self.model(source=frame, verbose=False, conf=0.5)
            #self.annoted_frame = np.array(results[0])
        #else:
            #self.annoted_frame = None

        #Frame para exibição
        #frame_show = self.annoted_frame if self.annoted_frame is not None else frame

        #Converte para imagem comprimida (JPEG)
        #ros_compressed_image = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
        self.camera_publishing.publish(frame)

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
