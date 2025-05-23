#include "cr_bt_orchestrator/custom_bt_nodes.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace cr
{
    namespace bt_nodes
    {
        // ------------------------------------------------------------------------------------------------------------------
        //                                       Service Wrapper - GET OBJECT INFO
        // ------------------------------------------------------------------------------------------------------------------
        GetObjectInfo::GetObjectInfo(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
            : BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>(instance_name, conf, params) {}

        BT::PortsList GetObjectInfo::providedPorts()
        {
            return BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>::providedBasicPorts({BT::InputPort<std::string>("object_id"),
                                                                                              BT::OutputPort<double>("object_center_x"),
                                                                                              BT::OutputPort<double>("object_center_y"),
                                                                                              BT::OutputPort<double>("object_center_z"),
                                                                                              BT::OutputPort<double>("object_size_x"),
                                                                                              BT::OutputPort<double>("object_size_y"),
                                                                                              BT::OutputPort<double>("object_size_z")});
        }

        bool GetObjectInfo::setRequest(typename cr_interfaces::srv::GetObjectInfo::Request::SharedPtr &request)
        {
            std::string object_id_str;
            if (!getInput("object_id", object_id_str))
            {
                RCLCPP_ERROR(logger(), "Missing input [object_id]");
                return false;
            }

            try
            {
                int tmp = std::stoi(object_id_str);
                if (tmp < 0 || tmp > std::numeric_limits<uint8_t>::max())
                {
                    RCLCPP_ERROR(logger(),
                                 "object_id out of range [0..%u]",
                                 static_cast<unsigned>(std::numeric_limits<uint8_t>::max()));
                    return false;
                }
                request->id = static_cast<uint8_t>(tmp);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(logger(),
                             "Invalid object_id '%s': %s",
                             object_id_str.c_str(), e.what());
                return false;
            }
            return true;
        }

        BT::NodeStatus GetObjectInfo::onResponseReceived(const typename cr_interfaces::srv::GetObjectInfo::Response::SharedPtr &response)
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
            setOutput("object_center_x", response->object_info.center.x);
            setOutput("object_center_y", response->object_info.center.y);
            setOutput("object_center_z", response->object_info.center.z);
            setOutput("object_size_x", response->object_info.size.x);
            setOutput("object_size_y", response->object_info.size.y);
            setOutput("object_size_z", response->object_info.size.z);
            RCLCPP_INFO(logger(),
                        "Received object info: center=[%.2f, %.2f, %.2f]",
                        response->object_info.center.x,
                        response->object_info.center.y,
                        response->object_info.center.z);
            return BT::NodeStatus::SUCCESS;
        }

        // ------------------------------------------------------------------------------------------------------------------
        //                                       Service Wrapper - FREEZE SCENE
        // ------------------------------------------------------------------------------------------------------------------
        FreezeScene::FreezeScene(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
            : BT::RosServiceNode<cr_interfaces::srv::FreezeScene>(instance_name, conf, params) {}

        bool FreezeScene::setRequest(typename Request::SharedPtr &request)
        {
            request->freeze = true;
            return true;
        }

        BT::NodeStatus FreezeScene::onResponseReceived(const typename Response::SharedPtr &response)
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
            RCLCPP_INFO(logger(), "The planning scene was successfully frozen.");
            return BT::NodeStatus::SUCCESS;
        }

        // ------------------------------------------------------------------------------------------------------------------
        //                                       Utility Node - LOG MESSAGE
        // ------------------------------------------------------------------------------------------------------------------
        LogMessage::LogMessage(const std::string &name, const BT::NodeConfiguration &config)
            : BT::SyncActionNode(name, config) {}

        BT::PortsList LogMessage::providedPorts()
        {
            return {BT::InputPort<std::string>("message")};
        }

        BT::NodeStatus LogMessage::tick()
        {
            auto msg = getInput<std::string>("message");
            if (!msg)
            {
                RCLCPP_ERROR(rclcpp::get_logger("LogMessageNode"), "Missing port [message]");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(rclcpp::get_logger("LogMessageNode"), "BT_LOG: %s",
                        msg.value().c_str());
            return BT::NodeStatus::SUCCESS;
        }

        // ------------------------------------------------------------------------------------------------------------------
        //                                        Condition Node - IsAreaSafe
        // ------------------------------------------------------------------------------------------------------------------
        IsAreaSafe::IsAreaSafe(const std::string &name, const BT::NodeConfig &config)
            : BT::ConditionNode(name, config)
        {

            // Recuperiamo il nodo ROS dalla blackboard
            auto ros_node = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");

            // Sottoscrizione al topic di sicurezza
            sub_ = ros_node->create_subscription<std_msgs::msg::Bool>(
                "/cr/human_near", rclcpp::QoS(10),
                [this](const std_msgs::msg::Bool::SharedPtr msg)
                { human_near_.store(msg->data, std::memory_order_relaxed); });
        }

        BT::PortsList IsAreaSafe::providedPorts() { return {}; }

        BT::NodeStatus IsAreaSafe::tick()
        {
            // Area sicura ↔ non c’è umano vicino
            return human_near_.load(std::memory_order_relaxed)
                       ? BT::NodeStatus::FAILURE
                       : BT::NodeStatus::SUCCESS;
        }

        // ------------------------------------------------------------------------------------------------------------------
        //                                       Attesa di Area Safe
        // ------------------------------------------------------------------------------------------------------------------
        EnsureAreaIsSafe::EnsureAreaIsSafe(const std::string &name, const BT::NodeConfig &config)
            : BT::CoroActionNode(name, config)
        {

            node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node"); // Assicurati che "ros_node" sia nella blackboard
            if (!node_)
            {
                throw BT::RuntimeError("Missing 'ros_node' in blackboard for EnsureAreaIsSafe");
            }

            // Sottoscrivi al topic di sicurezza
            // Assicurati che il topic e il QoS siano corretti
            safety_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
                "/cr/human_near", rclcpp::QoS(10),
                std::bind(&EnsureAreaIsSafe::safetyCallback, this, std::placeholders::_1));

            RCLCPP_INFO(node_->get_logger(), "[EnsureAreaIsSafe] Initialized.");
            // Potresti voler leggere lo stato iniziale della sicurezza qui se disponibile,
            // altrimenti si basa sul valore di default e sul primo callback.
        }

        BT::NodeStatus EnsureAreaIsSafe::tick()
        {
            if (area_is_currently_safe_.load(std::memory_order_relaxed))
            {
                if (first_tick_unsafe_logged_)
                { // Se prima era insicuro e ora è sicuro
                    RCLCPP_INFO(node_->get_logger(), "[EnsureAreaIsSafe] Area is NOW SAFE. Proceeding.");
                    first_tick_unsafe_logged_ = false; // Resetta il flag
                }
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                // Area non è sicura
                if (!first_tick_unsafe_logged_)
                {
                    RCLCPP_WARN(node_->get_logger(), "[EnsureAreaIsSafe] Area is UNSAFE. Pausing operation until area is safe...");
                    first_tick_unsafe_logged_ = true;
                }
                // Logga meno frequentemente per non inondare i log
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[EnsureAreaIsSafe] Still waiting for area to become safe...");
                return BT::NodeStatus::RUNNING; // Mantiene l'albero in attesa
            }
        }

        void EnsureAreaIsSafe::halt()
        {
            RCLCPP_INFO(node_->get_logger(), "[EnsureAreaIsSafe] Halted.");
            first_tick_unsafe_logged_ = false; // Resetta in caso di halt
            CoroActionNode::halt();            // Chiamata al metodo della classe base
        }

        void EnsureAreaIsSafe::safetyCallback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            bool previously_safe = area_is_currently_safe_.exchange(!msg->data, std::memory_order_relaxed);
            if (previously_safe && msg->data)
            {
                RCLCPP_WARN(node_->get_logger(), "[EnsureAreaIsSafe] Safety status changed: Area became UNSAFE.");
            }
            else if (!previously_safe && !msg->data)
            {
                RCLCPP_INFO(node_->get_logger(), "[EnsureAreaIsSafe] Safety status changed: Area became SAFE.");
                // Non è necessario fare nulla qui, il prossimo tick() lo rileverà.
            }
        }

    } // namespace bt_nodes
} // namespace cr
