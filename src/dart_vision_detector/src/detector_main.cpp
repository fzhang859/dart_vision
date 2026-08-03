#include "dart_vision_detector/detector_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dart_vision::detector::DetectorNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}
