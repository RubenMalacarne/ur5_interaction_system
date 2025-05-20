#ifndef CR_BT_PICK_PLACE__SET_GRIPPER_NODE_HPP_
#define CR_BT_PICK_PLACE__SET_GRIPPER_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <memory>
#include "cr_motion_core/motion_commander.hpp"

namespace cr::bt::pick_place
{

    class SetGripperNode : public BT::StatefulActionNode
    {
    public:
        SetGripperNode(const std::string &name,
                       const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts();

    private:
        // StatefulActionNode interface
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

        // own members
        double target_{0.0};
        std::shared_ptr<cr::motion_core::MotionCommander> commander_;
    };

} // namespace cr::bt::pick_place

#endif // CR_BT_PICK_PLACE__SET_GRIPPER_NODE_HPP_
