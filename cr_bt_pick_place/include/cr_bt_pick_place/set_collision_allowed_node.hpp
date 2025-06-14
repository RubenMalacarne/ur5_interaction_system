/**
 * @file set_collision_allowed_node.hpp
 * @brief BT node that invokes a ROS service to set collision allowances for a given object.
 *
 * This node calls the `AllowCollision` service to dynamically enable or disable
 * collision checking with a specific object, based on a given ID.
 *
 * Used in pick-and-place tasks to allow or prevent collision with held objects.
 */

#ifndef CR_BT_PICK_PLACE_NODES__SET_COLLISION_ALLOWED_NODE_HPP_
#define CR_BT_PICK_PLACE_NODES__SET_COLLISION_ALLOWED_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cr_interfaces/srv/allow_collision.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_service_node.hpp>

namespace cr::bt::pick_place::nodes
{

    /**
     * @class SetCollisionAllowedNode
     * @brief BT service node that sets collision allowance for an object ID using a ROS service.
     *
     * This node sends a request to `cr_interfaces/srv/AllowCollision` service,
     * using the inputs "object_id" and "is_allowed" from the blackboard.
     */
    class SetCollisionAllowedNode : public BT::RosServiceNode<cr_interfaces::srv::AllowCollision>
    {
    public:
        /**
         * @brief Constructor.
         *
         * @param instance_name Name of the BT node instance.
         * @param conf Node configuration.
         * @param params ROS-related service parameters.
         */
        SetCollisionAllowedNode(const std::string &instance_name,
                                const BT::NodeConfig &conf,
                                const BT::RosNodeParams &params);

        /**
         * @brief Declares the input ports required by this node.
         *
         * - object_id (int): ID of the object to modify collision state for.
         * - is_allowed (bool): Whether collisions should be allowed or not.
         */
        static BT::PortsList providedPorts();

        /**
         * @brief Populates the ROS service request using BT input ports.
         */
        bool setRequest(typename Request::SharedPtr &request) override;

        /**
         * @brief Processes the service response and determines the BT node status.
         */
        BT::NodeStatus onResponseReceived(const typename Response::SharedPtr &response) override;

    protected:
        /**
         * @brief Returns a logger named after the node instance.
         */
        rclcpp::Logger logger() { return rclcpp::get_logger(this->name()); }
    };

} // namespace cr::bt::pick_place::nodes

#endif // CR_BT_PICK_PLACE_NODES__SET_COLLISION_ALLOWED_NODE_HPP_
