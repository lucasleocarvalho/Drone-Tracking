#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ncnn/net.h"  // Inclua a biblioteca NCNN corretamente

using std::placeholders::_1;

class DetectionNode : public rclcpp::Node {
public:
    DetectionNode()
    : Node("detection_node")
    {
        // QoS profile: BEST_EFFORT + KEEP_LAST(1)
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
                       .best_effort().keep_last(1);

        // Inicializa câmera
        cap.open(0, cv::CAP_V4L2);
        cap.set(cv::CAP_PROP_FPS, 30);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

        double fps = cap.get(cv::CAP_PROP_FPS);
        RCLCPP_INFO(this->get_logger(), "FPS configurado na câmera: %.2f", fps);

        // Publisher de imagem
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/compressed", qos);

        // Carrega o modelo YOLO (usando NCNN)
        std::string package_path = ament_index_cpp::get_package_share_directory("drone_tracking");
        std::string model_path_param = package_path + "/net_train/weights/best.param";
        std::string model_path_bin = package_path + "/net_train/weights/best.bin";
        if (yolo_net.load_param(model_path_param.c_str()) || yolo_net.load_model(model_path_bin.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao carregar modelo YOLO (NCNN).");
        } else {
            RCLCPP_INFO(this->get_logger(), "Modelo YOLO carregado com sucesso.");
        }

        // Timer para 30 FPS
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
                                         std::bind(&DetectionNode::capture_callback, this));
    }

    ~DetectionNode() {
        cap.release();
    }

private:
    void capture_callback() {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao capturar frame da câmera");
            return;
        }

        frame_count++;
        if (frame_count % 10 == 0) {
            run_yolo(frame, annotated_frame);
        } else {
            annotated_frame = frame.clone();
        }

        // Converte imagem para CompressedImage
        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = this->now();
        msg.format = "jpeg";

        std::vector<uchar> buf;
        cv::imencode(".jpg", annotated_frame, buf);
        msg.data = buf;

        publisher_->publish(msg);
    }

    void run_yolo(const cv::Mat &frame, cv::Mat &output) {
        ncnn::Mat in = ncnn::Mat::from_pixels(frame.data, ncnn::Mat::PIXEL_BGR, frame.cols, frame.rows);

        ncnn::Extractor ex = yolo_net.create_extractor();
        ex.input("images", in);

        ncnn::Mat out;
        ex.extract("output", out);  // Ajuste conforme o nome da saída no modelo

        // Processamento simples do output (mock)
        output = frame.clone();
        for (int i = 0; i < out.h; i++) {
            const float* values = out.row(i);
            float x = values[2] * frame.cols;
            float y = values[3] * frame.rows;
            float w = values[4] * frame.cols;
            float h = values[5] * frame.rows;

            cv::Rect rect(x - w/2, y - h/2, w, h);
            cv::rectangle(output, rect, cv::Scalar(0, 255, 0), 2);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap;
    cv::Mat annotated_frame;

    int frame_count = 0;
    ncnn::Net yolo_net;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
