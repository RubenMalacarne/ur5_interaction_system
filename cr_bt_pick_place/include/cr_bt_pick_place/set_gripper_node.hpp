#ifndef CR_BT_PICK_PLACE__SET_GRIPPER_NODE_HPP_
#define CR_BT_PICK_PLACE__SET_GRIPPER_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <memory>
#include <rclcpp/rclcpp.hpp> // Necessario per rclcpp::Node::SharedPtr
#include <future> // Necessario per std::shared_future

#include "cr_motion_core/motion_commander.hpp" // Necessario per MotionCommander e MotionStatus

namespace cr::bt::pick_place
{

    class SetGripperNode : public BT::StatefulActionNode
    {
    public:
        SetGripperNode(const std::string &name,
                       const BT::NodeConfiguration &config);

       ~SetGripperNode() override;

        // Definisce le porte di input/output del nodo
        static BT::PortsList providedPorts();

    protected:
        // Implementazione dei metodi virtuali di StatefulActionNode
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

        // Future per tracciare il task asincrono gestito da MotionCommander
        std::shared_future<cr::motion_core::MotionStatus> gripper_task_future_;

        rclcpp::Node::SharedPtr nh_; // Nodo ROS per logging
        std::shared_ptr<cr::motion_core::MotionCommander> commander_;
    };

} // namespace cr::bt::pick_place

#endif // CR_BT_PICK_PLACE__SET_GRIPPER_NODE_HPP_
