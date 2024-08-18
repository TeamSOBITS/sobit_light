#include "rclcpp/rclcpp.hpp"
#include "sobit_light_library/sobit_light_wheel_controller.hpp"

int main(int argc, char *argv[]) {
    // ROS2の初期化
    rclcpp::init(argc, argv);

    // ノードの作成
    auto node = std::make_shared<sobit_light::WheelController>("sobit_light_test_control_wheel");

    // 車輪制御インスタンスの作成
    // `WheelController`クラスが提供するメソッドを利用します

    // 直線移動 (linear motion)
    node->controlWheelLinear(1.5);
    rclcpp::sleep_for(std::chrono::seconds(3));

    // 回転移動 (radian) (rotational motion: Radian)
    node->controlWheelRotateRad(1.5708);
    rclcpp::sleep_for(std::chrono::seconds(3));

    // 回転移動 (degree) (rotational motion: Degree)
    node->controlWheelRotateDeg(-90);
    rclcpp::sleep_for(std::chrono::seconds(3));

    // 直線移動 (linear motion)
    node->controlWheelLinear(-1.5);
    rclcpp::sleep_for(std::chrono::seconds(3));

    // ROS2の終了処理
    rclcpp::shutdown();

    return 0;
}
