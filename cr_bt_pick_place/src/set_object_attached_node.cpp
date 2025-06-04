#include "cr_bt_pick_place/set_object_attached_node.hpp"

using namespace cr::bt::pick_place::nodes;
using BT::NodeStatus;

SetObjectAttachedNode::SetObjectAttachedNode(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
    : BT::RosServiceNode<cr_interfaces::srv::AttachObject>(instance_name, conf, params) {}

BT::PortsList SetObjectAttachedNode::providedPorts()
{
    return BT::RosServiceNode<cr_interfaces::srv::AttachObject>::providedBasicPorts({
        BT::InputPort<int>("object_id"),
        BT::InputPort<bool>("attach")
    });
}

bool SetObjectAttachedNode::setRequest(typename Request::SharedPtr& request)
{
    int object_id;
    bool attach;

    // Leggi l'input "object_id"
    if (!getInput("object_id", object_id)) 
    {
        RCLCPP_ERROR(logger(), "Missing or invalid input [object_id]");
        return false;
    }

    // Leggi l'input "attach"
    if (!getInput("attach", attach)) 
    {
        RCLCPP_ERROR(logger(), "Missing or invalid input [attach]");
        return false;
    }

    // Assegna i valori alla richiesta
    request->object_id = static_cast<uint8_t>(object_id);
    request->attach = attach;

    return true;
}

BT::NodeStatus SetObjectAttachedNode::onResponseReceived(const typename Response::SharedPtr& response)
{
    if (!response)
    {
        RCLCPP_ERROR(logger(), "Service call failed (empty response)");
        return BT::NodeStatus::FAILURE;
    }
    if (!response->success)
    {
        RCLCPP_ERROR(logger(), "Service returned failure");
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger(), "Object attachment successfully set!");
    return BT::NodeStatus::SUCCESS;
}