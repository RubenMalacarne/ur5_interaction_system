#ifndef CR_BT_PICK_PLACE_NODES__SET_OBJECT_ATTACHED_NODE_HPP_
#define CR_BT_PICK_PLACE_NODES__SET_OBJECT_ATTACHED_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cr_interfaces/srv/attach_object.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_service_node.hpp>

namespace cr::bt::pick_place::nodes
{
    // ------------------------------------------------------------------------------------------------------------------
    //                                       Service Wrapper - SET OBJECT ATTACHED
    // ------------------------------------------------------------------------------------------------------------------
    class SetObjectAttachedNode: public BT::RosServiceNode<cr_interfaces::srv::AttachObject>
    {
    public:
        SetObjectAttachedNode(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params);

        static BT::PortsList providedPorts();

        bool setRequest(typename Request::SharedPtr& request) override;
        BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;

    protected:
        rclcpp::Logger logger() { return rclcpp::get_logger(this->name()); }
    };
}

#endif