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

        #Import da rede neural
        package_dir = get_package_share_directory('drone_tracking_py')
        model_path = os.path.join(package_dir, 'net_train', 'weights', 'best.pt')
        self.model = YOLO(model_path)
        self.model.export(format="ncnn")
        self.model = YOLO(os.path.join(package_dir, 'net_train', 'weights', 'best.pt'))

        #Definição de QoS pra reduzir delay na transmissão da câmera
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        #Definição de variáveis de detecção
        self.frame_c = 0
        self.annoted_frame = None

        #Definição das variáveis do filtro
        self.x = np.zeros((4, 1))                           #Matriz de variáveis (x, y, vx, vy)
        self.P = np.eye(4) * 1000                           #Incerteza inicial alta
        self.dt = 1/30.0                                    #Tempo discretizado para medições
        self.F = np.array([                                 #Essa matriz assume velocidade constante em regime permanente (matriz de transição)
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.H = np.array([                                 #Matriz de observação
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        self.Q = np.eye(4) * 0.1                            #Ruído do processo
        self.R = np.eye(2) * 5.0                            #Ruído do sensor


        # Subscriber para imagem comprimida
        self.camera_subscription = self.create_subscription(CompressedImage, '/camera/compressed', self.image_callback, qos)
    
    def image_callback(self, msg):
        # Converte imagem comprimida para cv2
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        results = None

        #Detecção com YOLO
        self.frame_c += 1
        if self.frame_c % 5 == 0:
            results = self.model(frame, verbose=False, conf=0.5)
            self.annoted_frame = np.array(results[0].plot())
        else:
            self.annoted_frame = None

        #Frame para exibição
        frame_show = self.annoted_frame if self.annoted_frame is not None else frame

        #Envio dos dados para o algoritmo de EKF
        pos = self.ekf(frame_show, results)
        
        x_pred, y_pred = int(pos[0, 0]), int(pos[1, 0])
        cv.circle(frame_show, (x_pred, y_pred), 5, (0, 255, 0), -1)

        cv.imshow('Detecção YOLO', frame_show)
        if cv.waitKey(1) == ord('q'):
            self.get_logger().info('Fechando janela')
            rclpy.shutdown()

    def ekf(self, frame, results):
        #Detecção do centro das bouding boxes
        centers = []
        if results is not None:
            boxes = results[0].boxes.xyxy
            for box in boxes:
                x1, y1, x2, y2 = box.tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                centers.append((cx, cy))
        
        #Para frames onde não há medição de YOLO
        if not centers:
            self.x = self.F @ self.x                        #Estado atualizado
            self.P = self.F @ self.P @ self.F.T + self.Q    #Covariancia atualizada
            return self.x
        
        cx, cy = centers[-1]                                #Rever essa lógica pq se houver mais de um drone dá problema
        
        #Etapa de predição
        x_pred = self.F @ self.x                            #Estado previsto
        P_pred = self.F @ self.P @ self.F.T + self.Q        #Covariancia prevista

        #Etapa de medição
        z = np.array([[cx], [cy]])                          #Medição atual (vinda da YOLO)
        y = z - self.H @ x_pred                             #Redíduo (diferença entre o que o filtro mediu e a medição da YOLO)
        S = self.H @ P_pred @ self.H.T + self.R             #Incerteza da diferença entre o que foi previsto e medido
        K = P_pred @ self.H.T @ np.linalg.inv(S)            #Ganho de Kalman

        #Etapa de atualização
        self.x = x_pred + K @ y
        self.P = (np.eye(4) - K @ self.H) @ P_pred
        return self.x

def main(args=None):
    rclpy.init(args=args)
    node = CameraShow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
