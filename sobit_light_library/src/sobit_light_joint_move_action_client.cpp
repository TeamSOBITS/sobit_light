#include <string>
#include <sstream>

#include "sobits_interfaces/action/move_joint.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace sobit_light{

class JointMoveActionClient : public rclcpp::Node
{
public:
  using MoveJoint = sobits_interfaces::action::MoveJoint;
  using GoalHandleMoveJoints = rclcpp_action::ClientGoalHandle<sobits_interfaces::action::MoveJoint>;

  explicit JointMoveActionClient(const rclcpp::NodeOptions & options)
  : Node("joint_move_action_client", options)
  {
    this->action_client_ = rclcpp_action::create_client<MoveJoint>(
        this,
        "move_joint");
    RCLCPP_INFO(this->get_logger(), "JointActionClient has been initialized.");

    this->timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&JointMoveActionClient::send_goal, this));

  }
  ~JointMoveActionClient()
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

    auto goal = MoveJoint::Goal();
    goal.target_joint_names = {"arm_shoulder_pitch_joint"};
    goal.target_joint_rad = {1.0};
    goal.time_allowance = rclcpp::Duration(5, 0);

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<MoveJoint>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&JointMoveActionClient::goal_response_callback_joint_action_client, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&JointMoveActionClient::feedback_callback_joint_action_client, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&JointMoveActionClient::result_callback_joint_action_client, this, std::placeholders::_1);
    this->action_client_->async_send_goal(goal, send_goal_options);
  }

private:
  rclcpp_action::Client<MoveJoint>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback_joint_action_client(const GoalHandleMoveJoints::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback_joint_action_client(
    GoalHandleMoveJoints::SharedPtr,
    const std::shared_ptr<const MoveJoint::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Feedback: ";
    for (size_t i = 0; i < feedback->current_joint_names.size(); i++) {
      ss << feedback->current_joint_names[i] << "=" << feedback->current_joint_rad[i] << ", ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback_joint_action_client(const GoalHandleMoveJoints::WrappedResult & result)
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
    RCLCPP_INFO(this->get_logger(), "Result received with total elapsed time: %d.%09d",
        result.result->total_elapsed_time.sec, result.result->total_elapsed_time.nanosec);
    
    rclcpp::shutdown();
  }
}; // class JointMoveActionClient

} // namespace sobit_light

RCLCPP_COMPONENTS_REGISTER_NODE(sobit_light::JointMoveActionClient)
