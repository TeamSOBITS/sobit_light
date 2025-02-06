#include <string>
#include <sstream>

#include "sobits_interfaces/action/move_hand_to_target_coord.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace sobit_light{

class JointCoordActionClient : public rclcpp::Node
{
public:
  using MoveHandToTargetCoord = sobits_interfaces::action::MoveHandToTargetCoord;
  using GoalHandleMoveHandToCoord = rclcpp_action::ClientGoalHandle<sobits_interfaces::action::MoveHandToTargetCoord>;

  explicit JointCoordActionClient(const rclcpp::NodeOptions & options)
  : Node("joint_coord_action_client", options)
  {
    this->action_client_ = rclcpp_action::create_client<MoveHandToTargetCoord>(
        this,
        "move_hand_to_coord");
    RCLCPP_INFO(this->get_logger(), "JointActionClient has been initialized.");

    this->timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&JointCoordActionClient::send_goal, this));

  }
  ~JointCoordActionClient()
  {
    this->action_client_.reset();
    RCLCPP_INFO(this->get_logger(), "JointActionClient has been terminated.");
  }

  void send_goal()
  {
    this->timer_->cancel();
    if (!this->action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    geometry_msgs::msg::TransformStamped target_coord;
    target_coord.header.frame_id = "hand_end_effector_link";
    target_coord.transform.translation.x = 0.3;
    target_coord.transform.translation.y = 0.0;
    target_coord.transform.translation.z = 0.1;

    auto goal = MoveHandToTargetCoord::Goal();
    goal.target_coord = target_coord;
    goal.time_allowance = rclcpp::Duration(5, 0);

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<MoveHandToTargetCoord>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&JointCoordActionClient::goal_response_callback_joint_action_client, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&JointCoordActionClient::feedback_callback_joint_action_client, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&JointCoordActionClient::result_callback_joint_action_client, this, std::placeholders::_1);
    this->action_client_->async_send_goal(goal, send_goal_options);
  }

private:
  rclcpp_action::Client<MoveHandToTargetCoord>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback_joint_action_client(const GoalHandleMoveHandToCoord::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback_joint_action_client(
    GoalHandleMoveHandToCoord::SharedPtr,
    const std::shared_ptr<const MoveHandToTargetCoord::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Feedback: ";
    ss << "current_state: " << feedback->current_state << ", ";
    ss << "distance_to_target: " << feedback->distance_to_target;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback_joint_action_client(const GoalHandleMoveHandToCoord::WrappedResult & result)
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
}; // class JointCoordActionClient

} // namespace sobit_light

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_light::JointCoordActionClient)
