#include "behaviortree_ros2/bt_action_node.hpp"
#include "ur_controller_msgs/action/universal_robot_control.hpp"

using namespace BT;

class SleepAction : public RosActionNode<ur_controller_msgs::action::UniversalRobotControl>
{
public:
  SleepAction(const std::string &name, const NodeConfig &conf,
              const RosNodeParams &params)
      : RosActionNode<ur_controller_msgs::action::UniversalRobotControl>("bt_action", conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    PortsList basic = {};

    basic.insert(InputPort<std::string>("action_name", "", "Action server name"));
    basic.insert(InputPort<unsigned>("msec"));
    basic.insert(InputPort<std::string>("robot_position"));

    return basic;
  }

  bool setGoal(Goal &goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult &wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
