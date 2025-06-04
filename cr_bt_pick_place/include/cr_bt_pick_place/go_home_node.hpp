#ifndef CR_BT_PICK_PLACE_NODES__GO_HOME_NODE_HPP_
#define CR_BT_PICK_PLACE_NODES__GO_HOME_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <future>

#include <cr_motion_core/motion_commander.hpp>

namespace cr::bt::pick_place::nodes
{

    class GoHomeNode : public BT::StatefulActionNode
    {
    public:
        GoHomeNode(const std::string &name, const BT::NodeConfiguration &config);

        ~GoHomeNode() override;

        static inline BT::PortsList providedPorts() { return {}; }


    protected:
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

        std::shared_future<cr::motion_core::MotionStatus> arm_task_future_;

        rclcpp::Node::SharedPtr nh_;
        std::shared_ptr<cr::motion_core::MotionCommander> commander_;
    };

} // namespace cr::bt::pick_place::nodes

#endif