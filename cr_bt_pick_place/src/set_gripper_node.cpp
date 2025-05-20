#include "cr_bt_pick_place/set_gripper_node.hpp"
#include <behaviortree_cpp/bt_factory.h>

using namespace cr::bt::pick_place;
using BT::NodeStatus;

SetGripperNode::SetGripperNode(const std::string &name,
                               const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
    // nulla da fare qui: il commander arriva in onStart()
}

BT::PortsList SetGripperNode::providedPorts()
{
    return {
        BT::InputPort<double>("target", "apertura gripper [rad]"),
        BT::InputPort<std::shared_ptr<cr::motion_core::MotionCommander>>(
            "commander", "MotionCommander dalla Blackboard")};
}

NodeStatus SetGripperNode::onStart()
{
    // 1) Leggi target
    if (!getInput("target", target_))
    {
        throw BT::RuntimeError("SetGripperNode: missing port [target]");
    }
    // 2) Leggi commander
    if (!getInput("commander", commander_))
    {
        throw BT::RuntimeError("SetGripperNode: missing port [commander]");
    }

    // 3) Invia comando asincrono
    bool sent = commander_->sendSetGripper(target_);
    return sent ? NodeStatus::RUNNING
                : NodeStatus::FAILURE;
}

NodeStatus SetGripperNode::onRunning()
{
    // 4) Controlla lo status
    int status = commander_->setGripperStatus();
    switch (status)
    {
    case 0:
        return NodeStatus::RUNNING;
    case 1:
        return NodeStatus::SUCCESS;
    default:
        return NodeStatus::FAILURE;
    }
}

void SetGripperNode::onHalted()
{
    // Se interrompiamo, annulla il comando in corso
    commander_->cancelSetGripper();
}

// Registrazione del nodo come plugin
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<SetGripperNode>("SetGripper");
}
