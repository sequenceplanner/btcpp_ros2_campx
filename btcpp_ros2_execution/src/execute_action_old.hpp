#include "behaviortree_ros2/bt_action_node.hpp"
#include "ur_controller_msgs/action/universal_robot_control.hpp"
#include "btcpp_ros2_interfaces/action/execute_action.hpp"


using namespace BT;

class BehaviorTreeAction : public RosActionNode<btcpp_ros2_interfaces::action::ExecuteAction>
{
public:
  BehaviorTreeAction(const std::string &name, const NodeConfig &conf,
              const RosNodeParams &params)
      : RosActionNode<btcpp_ros2_interfaces::action::ExecuteAction>("bt_action_service", conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    PortsList basic = {};

    // define input / output ports here
    // NOTE: have to be the same as in the behavior tree .xml files
    // NOTE: action_name is mandatory as it connects to the ros action nodes
    basic.insert(InputPort<std::string>("action_name", "", "Action server name"));
    // basic.insert(InputPort<unsigned>("msec"));
    basic.insert(InputPort<std::string>("robot_position"));

    return basic;
  }

  bool setGoal(Goal &goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult &wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};
