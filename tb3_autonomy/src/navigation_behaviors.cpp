#include "navigation_behaviors.h"
#include "yaml-cpp/yaml.h"
#include <string>

GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/navigate_to_pose");
  done_flag_ = false;
  distance_remaining_ = -1.0;
}

BT::PortsList GoToPose::providedPorts()
{
  return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart()
{
  // Get location key from port and read YAML file
  BT::Optional<std::string> loc = getInput<std::string>("loc");
  const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

  YAML::Node locations = YAML::LoadFile(location_file);

  std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

  if (!action_client_ptr_->wait_for_action_server(std::chrono::seconds(2)))
  {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Action server /navigate_to_pose not available");
      return BT::NodeStatus::FAILURE;
  }

  // setup action client
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
        std::bind(&GoToPose::nav_to_pose_feedback_callback, this,
                   std::placeholders::_1, std::placeholders::_2);

  // make pose
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = pose[0];
  goal_msg.pose.pose.position.y = pose[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, pose[2]);
  q.normalize(); // todo: why?
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // send pose
  done_flag_ = false;
  distance_remaining_ = -1.0;
  last_feedback_time_ = node_ptr_->now();

  action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2\n");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
  if (done_flag_)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name().c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // Stuck detection: if too long has passed since the last feedback message,
  // assume the robot is stuck and cancel the goal.
  double seconds_since_feedback = (node_ptr_->now() - last_feedback_time_).seconds();
  if (seconds_since_feedback > FEEDBACK_TIMEOUT_SEC)
  {
      RCLCPP_WARN(node_ptr_->get_logger(),
                  "[%s] No feedback for %.1f seconds — assuming stuck, cancelling goal",
                  this->name().c_str(), seconds_since_feedback);
      action_client_ptr_->async_cancel_all_goals();
      return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
  // If there is a result, we consider navigation completed.
  // bt_navigator only sends an empty message without status. Idk why though.

  if (result.result)
  {
    done_flag_ = true;
  }
}

void GoToPose::nav_to_pose_feedback_callback(
    GoalHandleNav::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    last_feedback_time_ = node_ptr_->now();
    distance_remaining_ = feedback->distance_remaining;

    RCLCPP_INFO(node_ptr_->get_logger(),
                "[%s] Distance remaining: %.2f m",
                this->name().c_str(), distance_remaining_);
}