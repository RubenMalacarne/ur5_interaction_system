#include "cr_bt_orchestrator/custom_bt_nodes.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace cr
{
    namespace bt_nodes
    {
        // ------------------------------------------------------------------------------------------------------------------
        //                                       Utility Node - COMPUTE PRE-APPROACH Z
        // ------------------------------------------------------------------------------------------------------------------
        ComputePreApproachZ::ComputePreApproachZ(const std::string &name, const BT::NodeConfiguration &config)
            : BT::SyncActionNode(name, config)
        {
            // Ottieni il nodo ROS dalla blackboard (come hai fatto negli altri nodi)
            if (!config.blackboard->get("ros_node", nh_))
            {
                throw BT::RuntimeError("ComputePreApproachZ: missing 'ros_node' on blackboard");
            }
        }

        BT::PortsList ComputePreApproachZ::providedPorts()
        {
            return {
                BT::InputPort<cr_interfaces::msg::ObjectInfo>("object_info"),
                BT::InputPort<double>("pre_approach_distance"),
                BT::OutputPort<double>("target_z")};
        }

        BT::NodeStatus ComputePreApproachZ::tick()
        {
            cr_interfaces::msg::ObjectInfo object_info;
            double pre_approach_distance;

            // Leggi i valori dagli input ports
            if (!getInput("object_info", object_info))
            {
                RCLCPP_ERROR(nh_->get_logger(), "ComputePreApproachZ: missing input [object_info]");
                return BT::NodeStatus::FAILURE;
            }
            if (!getInput("pre_approach_distance", pre_approach_distance))
            {
                RCLCPP_ERROR(nh_->get_logger(), "ComputePreApproachZ: missing input [pre_approach_distance]");
                return BT::NodeStatus::FAILURE;
            }

            // Calcola la coordinata Z target
            double target_z = object_info.center.z + object_info.size.z / 2.0 + pre_approach_distance;

            // Scrivi il risultato nell'output port
            setOutput("target_z", target_z);

            RCLCPP_INFO(nh_->get_logger(), "ComputePreApproachZ: Calculated target_z = %.3f", target_z);
            return BT::NodeStatus::SUCCESS;
        }

        // ------------------------------------------------------------------------------------------------------------------
        //                                       Utility Node - COMPUTE OBJECT CENTER XY
        // ------------------------------------------------------------------------------------------------------------------
        ComputeObjectCenterXY::ComputeObjectCenterXY(const std::string &name, const BT::NodeConfiguration &config)
            : BT::SyncActionNode(name, config)
        {
            if (!config.blackboard->get("ros_node", nh_))
            {
                throw BT::RuntimeError("ComputeObjectCenterXY: missing 'ros_node' on blackboard");
            }
        }

        BT::PortsList ComputeObjectCenterXY::providedPorts()
        {
            return {
                BT::InputPort<cr_interfaces::msg::ObjectInfo>("object_info"),
                BT::OutputPort<double>("target_x"),
                BT::OutputPort<double>("target_y")};
        }

        BT::NodeStatus ComputeObjectCenterXY::tick()
        {
            cr_interfaces::msg::ObjectInfo object_info;

            // Leggi i valori dagli input ports
            if (!getInput("object_info", object_info))
            {
                RCLCPP_ERROR(nh_->get_logger(), "ComputeObjectCenterXY: missing input [object_info]");
                return BT::NodeStatus::FAILURE;
            }

            // Calcola le coordinate X e Y target
            double target_x = object_info.center.x;
            double target_y = object_info.center.y;

            // Scrivi il risultato negli output ports
            setOutput("target_x", target_x);
            setOutput("target_y", target_y);

            RCLCPP_INFO(nh_->get_logger(), "ComputeObjectCenterXY: Calculated target_x = %.3f, target_y = %.3f", target_x, target_y);
            return BT::NodeStatus::SUCCESS;
        }

        // ------------------------------------------------------------------------------------------------------------------
        //                                       Utility Node - COMPUTE APPROACH Z
        // ------------------------------------------------------------------------------------------------------------------
        ComputeApproachZ::ComputeApproachZ(const std::string &name, const BT::NodeConfiguration &config)
            : BT::SyncActionNode(name, config)
        {
            if (!config.blackboard->get("ros_node", nh_))
            {
                throw BT::RuntimeError("ComputeApproachZ: missing 'ros_node' on blackboard");
            }
        }

        BT::PortsList ComputeApproachZ::providedPorts()
        {
            return {
                BT::InputPort<cr_interfaces::msg::ObjectInfo>("object_info"),
                BT::InputPort<double>("approach_distance"),
                BT::OutputPort<double>("target_z")};
        }

        BT::NodeStatus ComputeApproachZ::tick()
        {
            cr_interfaces::msg::ObjectInfo object_info;
            double approach_distance;

            // Leggi i valori dagli input ports
            if (!getInput("object_info", object_info))
            {
                RCLCPP_ERROR(nh_->get_logger(), "ComputeApproachZ: missing input [object_info]");
                return BT::NodeStatus::FAILURE;
            }
            if (!getInput("approach_distance", approach_distance))
            {
                RCLCPP_ERROR(nh_->get_logger(), "ComputeApproachZ: missing input [approach_distance]");
                return BT::NodeStatus::FAILURE;
            }

            // Calcola la coordinata Z target
            double target_z = object_info.center.z + object_info.size.z / 2.0 + approach_distance;

            // Scrivi il risultato nell'output port
            setOutput("target_z", target_z);

            RCLCPP_INFO(nh_->get_logger(), "ComputeApproachZ: Calculated target_z = %.3f", target_z);
            return BT::NodeStatus::SUCCESS;
        }
    } // namespace bt
} // namespace cr
