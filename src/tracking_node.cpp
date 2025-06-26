#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class DetectionNode : public rclcpp::Node {
public:
    DetectionNode()
    : Node("detection_node"), frame_count_(0)
    {
        // QoS para reduzir delay
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos.keep_last(1);

        // Inicializa câmera
        cap_.open(0, cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FPS, 30);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Não foi possível abrir a câmera.");
            rclcpp::shutdown();
            return;
        }

        double fps = cap_.get(cv::CAP_PROP_FPS);
        RCLCPP_INFO(this->get_logger(), "FPS configurado na câmera: %.2f", fps);

        // Publisher de imagem comprimida
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/compressed", qos);

        // Timer para captura
        timer_ = this->create_wall_timer(33ms, std::bind(&DetectionNode::capture_callback, this));
    }

    ~DetectionNode() {
        cap_.release();
    }

private:
    void capture_callback() {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao capturar frame da câmera.");
            return;
        }

        // TODO: Inserir detecção com YOLO/NCNN aqui se desejado

        // Conversão OpenCV -> ROS2 CompressedImage
        auto msg = sensor_msgs::msg::CompressedImage();
        msg.header.stamp = this->now();
        msg.format = "jpeg";

        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf);
        msg.data = buf;

        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    size_t frame_count_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
