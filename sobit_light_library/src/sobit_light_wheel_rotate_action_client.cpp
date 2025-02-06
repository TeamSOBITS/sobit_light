#include <string>
#include <sstream>

#include "sobits_interfaces/action/move_wheel_rotate.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace sobit_light{

class WheelRotateActionClient : public rclcpp::Node
{
public:
  using MoveWheelRotate = sobits_interfaces::action::MoveWheelRotate;
  using GoalHandleMoveWheelRotate = rclcpp_action::ClientGoalHandle<sobits_interfaces::action::MoveWheelRotate>;

  explicit WheelRotateActionClient(const rclcpp::NodeOptions & options)
  : Node("wheel_rotate_action_client", options)
  {
    this->action_client_ = rclcpp_action::create_client<MoveWheelRotate>(
        this,
        "move_wheel_rotate");
    RCLCPP_INFO(this->get_logger(), "WheelActionClient has been initialized.");

    this->timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&WheelRotateActionClient::send_goal, this));

  }
  ~WheelRotateActionClient()
  {
    this->action_client_.reset();
    RCLCPP_INFO(this->get_logger(), "WheelActionClient has been terminated.");
  }

  void send_goal()
  {
    this->timer_->cancel();
    if (!this->action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }


    auto goal = MoveWheelRotate::Goal();
    goal.target_yaw = 1.57;
    goal.time_allowance = rclcpp::Duration(5, 0);

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<MoveWheelRotate>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&WheelRotateActionClient::goal_response_callback_joint_action_client, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&WheelRotateActionClient::feedback_callback_joint_action_client, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&WheelRotateActionClient::result_callback_joint_action_client, this, std::placeholders::_1);
    this->action_client_->async_send_goal(goal, send_goal_options);
  }

private:
  rclcpp_action::Client<MoveWheelRotate>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback_joint_action_client(const GoalHandleMoveWheelRotate::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback_joint_action_client(
    GoalHandleMoveWheelRotate::SharedPtr,
    const std::shared_ptr<const MoveWheelRotate::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Feedback: ";
    ss << "current_point: (" << feedback->current_point.x << ", " << feedback->current_point.y << ", " << feedback->current_point.z << ")";
    ss << ", move_time: " << feedback->move_time.sec << "." << feedback->move_time.nanosec;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback_joint_action_client(const GoalHandleMoveWheelRotate::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    RCLCPP_INFO(this->get_logger(), "Result: %s, Message: %s",
        result.result->success ? "Succeeded" : "Failed", result.result->message.c_str());

    rclcpp::shutdown();
  }
}; // class WheelRotateActionClient

} // namespace sobit_light

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_light::WheelRotateActionClient)
