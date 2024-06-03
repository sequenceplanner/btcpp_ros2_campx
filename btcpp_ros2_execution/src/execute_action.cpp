#include "execute_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool SleepAction::setGoal(RosActionNode::Goal& goal)
{
  auto pos = getInput<std::string>("robot_position");
  goal.command = pos.value();
  return true;
}

NodeStatus SleepAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "%s: onResultReceived. Done = %s", name().c_str(),
              wr.result->success ? "true" : "false");

  return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

NodeStatus SleepAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void SleepAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

// Plugin registration.
// The class SleepAction will self register with name  "SleepAction".
CreateRosNodePlugin(SleepAction, "SleepAction");
