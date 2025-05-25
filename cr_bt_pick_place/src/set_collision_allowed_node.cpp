#include "cr_bt_pick_place/set_collision_allowed_node.hpp"

using namespace cr::bt::pick_place::nodes;
using BT::NodeStatus;

SetCollisionAllowedNode::SetCollisionAllowedNode(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
    : BT::RosServiceNode<cr_interfaces::srv::AllowCollision>(instance_name, conf, params) {}

BT::PortsList SetCollisionAllowedNode::providedPorts()
{
    return BT::RosServiceNode<cr_interfaces::srv::AllowCollision>::providedBasicPorts({
        BT::InputPort<int>("object_id"),
        BT::InputPort<bool>("is_allowed")
    });
}

bool SetCollisionAllowedNode::setRequest(typename Request::SharedPtr& request)
{
    int object_id;
    bool is_allowed;

    // Leggi l'input "object_id"
    if (!getInput("object_id", object_id)) 
    {
        RCLCPP_ERROR(logger(), "Missing or invalid input [object_id]");
        return false;
    }

    // Leggi l'input "is_allowed"
    if (!getInput("is_allowed", is_allowed)) 
    {
        RCLCPP_ERROR(logger(), "Missing or invalid input [is_allowed]");
        return false;
    }

    // Assegna i valori alla richiesta
    request->object_id = static_cast<uint8_t>(object_id);
    request->is_allowed = is_allowed;

    return true;
}

BT::NodeStatus SetCollisionAllowedNode::onResponseReceived(const typename Response::SharedPtr& response)
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
    RCLCPP_INFO(logger(), "Collision state successfully updated!");
    return BT::NodeStatus::SUCCESS;
}