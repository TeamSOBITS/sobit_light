#include "rclcpp/rclcpp.hpp"
#include "sobit_light_library/sobit_light_wheel_controller.hpp"

int main(int argc, char *argv[]) {
  // ROS2の初期化
  rclcpp::init(argc, argv);

  // ノードの作成
  auto node = std::make_shared<sobit_light::WheelController>();

  // 車輪制御インスタンスの作成
  // `WheelController`クラスが提供するメソッドを利用します

  // 直線移動 (linear motion)
  node->controlWheelLinear(0.5);

  // 回転移動 (radian) (rotational motion: Radian)
  node->controlWheelRotateRad(-1.5708);

  // 回転移動 (degree) (rotational motion: Degree)
  node->controlWheelRotateDeg(90);

  // 直線移動 (linear motion)
  node->controlWheelLinear(-0.5);

  // ノードの終了処理 (Node termination)
  node.reset();

  // ROS2の終了処理 (ROS2 termination)
  rclcpp::shutdown();

  return 0;
}
