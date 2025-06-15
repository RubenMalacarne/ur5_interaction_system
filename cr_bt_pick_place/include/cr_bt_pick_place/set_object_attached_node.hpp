/**
 * @file set_object_attached_node.hpp
 * @brief BT node that invokes a ROS service to attach or detach an object to/from the robot.
 *
 * This Behavior Tree node wraps a call to a ROS 2 service to set whether
 * a specific object should be considered attached (e.g. gripped) or detached.
 */

#ifndef CR_BT_PICK_PLACE_NODES__SET_OBJECT_ATTACHED_NODE_HPP_
#define CR_BT_PICK_PLACE_NODES__SET_OBJECT_ATTACHED_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cr_interfaces/srv/attach_object.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_service_node.hpp>

namespace cr::bt::pick_place::nodes
{

    /**
     * @class SetObjectAttachedNode
     * @brief BT node that invokes the AttachObject service to set an object's attachment state.
     *
     * Ports:
     * - Input<int>: `object_id` – the ID of the object to attach or detach.
     * - Input<bool>: `attach` – true to attach, false to detach.
     */
    class SetObjectAttachedNode : public BT::RosServiceNode<cr_interfaces::srv::AttachObject>
    {
    public:
        /**
         * @brief Constructor.
         * @param instance_name Name of the node instance.
         * @param conf BT configuration.
         * @param params ROS node parameters.
         */
        SetObjectAttachedNode(const std::string &instance_name,
                              const BT::NodeConfig &conf,
                              const BT::RosNodeParams &params);

        /**
         * @brief Defines input ports: object_id and attach flag.
         */
        static BT::PortsList providedPorts();

        /**
         * @brief Builds the service request based on BT input ports.
         */
        bool setRequest(typename Request::SharedPtr &request) override;

        /**
         * @brief Handles the response from the service.
         */
        BT::NodeStatus onResponseReceived(const typename Response::SharedPtr &response) override;

    protected:
        /// Returns a logger scoped to the node's name.
        rclcpp::Logger logger() { return rclcpp::get_logger(this->name()); }
    };

} // namespace cr::bt::pick_place::nodes

#endif // CR_BT_PICK_PLACE_NODES__SET_OBJECT_ATTACHED_NODE_HPP_
