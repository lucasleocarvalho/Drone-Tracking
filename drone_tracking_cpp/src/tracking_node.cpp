#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>


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
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir a câmera");
            rclcpp::shutdown();
            return;
        }
        cap.set(cv::CAP_PROP_FPS, 30);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

        double fps = cap.get(cv::CAP_PROP_FPS);
        RCLCPP_INFO(this->get_logger(), "FPS configurado na câmera: %.2f", fps);

        // Publisher de imagem
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/compressed", qos);

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

        // Converte imagem para CompressedImage
        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = this->now();
        msg.format = "jpeg";

        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf);
        msg.data = buf;

        publisher_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
