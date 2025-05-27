#include "cr_bt_orchestrator/nodes.hpp"

#include <functional>
#include <limits>
#include <stdexcept>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace cr::bt::orchestrator::nodes
{

    // ------------------------------------------------------------------------------------------------------------------
    //                                       Service Wrapper - GET OBJECT INFO
    // ------------------------------------------------------------------------------------------------------------------

    GetObjectInfo::GetObjectInfo(
        const std::string &instance_name,
        const BT::NodeConfig &conf,
        const BT::RosNodeParams &params) : BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>(instance_name, conf, params)
    {
    }

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

    bool GetObjectInfo::setRequest(typename Request::SharedPtr &request)
    {
        std::string object_id_str;
        if (!getInput("object_id", object_id_str))
        {
            BT_ACTION_LOG_ERROR("Missing input [object_id]");
            return false;
        }

        try
        {
            const int tmp = std::stoi(object_id_str);
            if (tmp < 0 || tmp > std::numeric_limits<uint8_t>::max())
            {
                BT_ACTION_LOG_ERROR(
                    "object_id out of range [0..%u]",
                    static_cast<unsigned>(std::numeric_limits<uint8_t>::max()));
                return false;
            }
            request->id = static_cast<uint8_t>(tmp);
        }
        catch (const std::exception &e)
        {
            BT_ACTION_LOG_ERROR(
                "Invalid object_id '%s': %s",
                object_id_str.c_str(),
                e.what());
            return false;
        }

        return true;
    }

    BT::NodeStatus GetObjectInfo::onResponseReceived(const typename Response::SharedPtr &response)
    {
        if (!response)
        {
            BT_ACTION_LOG_ERROR("Service call failed (empty response)");
            return BT::NodeStatus::FAILURE;
        }

        if (!response->success)
        {
            BT_ACTION_LOG_ERROR("Service returned failure");
            return BT::NodeStatus::FAILURE;
        }

        setOutput("object_center_x", response->object_info.center.x);
        setOutput("object_center_y", response->object_info.center.y);
        setOutput("object_center_z", response->object_info.center.z);
        setOutput("object_size_x", response->object_info.size.x);
        setOutput("object_size_y", response->object_info.size.y);
        setOutput("object_size_z", response->object_info.size.z);

        BT_ACTION_LOG_INFO(
            "Received object info: center=[%.2f, %.2f, %.2f]",
            response->object_info.center.x,
            response->object_info.center.y,
            response->object_info.center.z);

        return BT::NodeStatus::SUCCESS;
    }

    // ------------------------------------------------------------------------------------------------------------------
    //                                       Service Wrapper - FREEZE SCENE
    // ------------------------------------------------------------------------------------------------------------------

    FreezeScene::FreezeScene(
        const std::string &instance_name,
        const BT::NodeConfig &conf,
        const BT::RosNodeParams &params) : BT::RosServiceNode<cr_interfaces::srv::FreezeScene>(instance_name, conf, params)
    {
    }

    BT::PortsList FreezeScene::providedPorts()
    {
        return BT::RosServiceNode<cr_interfaces::srv::FreezeScene>::providedBasicPorts({BT::InputPort<bool>("freeze")});
    }

    bool FreezeScene::setRequest(typename Request::SharedPtr &request)
    {
        bool freeze;
        if (!getInput("freeze", freeze))
        {
            BT_ACTION_LOG_ERROR("Missing or invalid input [freeze]");
            return false;
        }

        request->freeze = freeze;
        return true;
    }

    BT::NodeStatus FreezeScene::onResponseReceived(const typename Response::SharedPtr &response)
    {
        if (!response)
        {
            BT_ACTION_LOG_ERROR("Service call failed (empty response)");
            return BT::NodeStatus::FAILURE;
        }

        if (!response->success)
        {
            BT_ACTION_LOG_ERROR("Service returned failure");
            return BT::NodeStatus::FAILURE;
        }

        BT_ACTION_LOG_INFO("Planning scene successfully frozen");
        return BT::NodeStatus::SUCCESS;
    }

    // ------------------------------------------------------------------------------------------------------------------
    //                                        Condition Node - IsAreaSafe
    // ------------------------------------------------------------------------------------------------------------------

    IsAreaSafe::IsAreaSafe(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        auto ros_node = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
        if (!ros_node)
        {
            throw BT::RuntimeError("Missing 'ros_node' in blackboard for IsAreaSafe");
        }

        safety_subscription_ = ros_node->create_subscription<std_msgs::msg::Bool>(
            "/cr/human_near",
            rclcpp::QoS(10),
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                human_near_.store(msg->data, std::memory_order_relaxed);
            });
    }

    BT::PortsList IsAreaSafe::providedPorts()
    {
        return {};
    }

    BT::NodeStatus IsAreaSafe::tick()
    {
        return human_near_.load(std::memory_order_relaxed)
                   ? BT::NodeStatus::FAILURE
                   : BT::NodeStatus::SUCCESS;
    }

    void IsAreaSafe::updateSafetyStatus(bool is_safe)
    {
        human_near_.store(!is_safe, std::memory_order_relaxed);
    }

    // ------------------------------------------------------------------------------------------------------------------
    //                                       Condition Node - IsStopRequested
    // ------------------------------------------------------------------------------------------------------------------
    IsStopRequested::IsStopRequested(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        auto ros_node = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
        if (!ros_node)
        {
            throw BT::RuntimeError("Missing 'ros_node' in blackboard for IsStopRequested");
        }

        stop_command_sub_ = ros_node->create_subscription<std_msgs::msg::Bool>(
            "/cr/stop_command",
            rclcpp::QoS(10),
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                stop_requested_.store(msg->data, std::memory_order_relaxed);
            });
    }

    BT::PortsList IsStopRequested::providedPorts()
    {
        return {};
    }

    BT::NodeStatus IsStopRequested::tick()
    {
        return stop_requested_.load(std::memory_order_relaxed)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }

    void IsStopRequested::updateStopRequestedStatus(bool is_stop_requested)
    {
        stop_requested_.store(is_stop_requested, std::memory_order_relaxed);
    }

    // ------------------------------------------------------------------------------------------------------------------
    //                                       Condition Node - IsPauseRequested
    // ------------------------------------------------------------------------------------------------------------------
    IsPauseRequested::IsPauseRequested(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        auto ros_node = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
        if (!ros_node)
        {
            throw BT::RuntimeError("Missing 'ros_node' in blackboard for IsStopRequested");
        }

        pause_command_sub_ = ros_node->create_subscription<std_msgs::msg::Bool>(
            "/cr/pause_command",
            rclcpp::QoS(10),
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                pause_requested_.store(msg->data, std::memory_order_relaxed);
            });
    }

    BT::PortsList IsPauseRequested::providedPorts()
    {
        return {};
    }

    BT::NodeStatus IsPauseRequested::tick()
    {
        return pause_requested_.load(std::memory_order_relaxed)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }

    void IsPauseRequested::updatePauseRequestedStatus(bool is_stop_requested)
    {
        pause_requested_.store(is_stop_requested, std::memory_order_relaxed);
    }

    // ------------------------------------------------------------------------------------------------------------------
    //                                       Action Node - WaitForTheGoAhead
    // ------------------------------------------------------------------------------------------------------------------

    WaitForTheGoAhead::WaitForTheGoAhead(const std::string &name,
                                         const BT::NodeConfig &config)
        : BT::CoroActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
        if (!node_)
        {
            throw BT::RuntimeError("Missing 'ros_node' in blackboard for WaitForTheGoAhead");
        }

        safety_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/cr/human_near",
            rclcpp::QoS(10),
            std::bind(&WaitForTheGoAhead::safetyCallback, this, std::placeholders::_1));

        pause_command_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/cr/pause_command",
            rclcpp::QoS(10),
            std::bind(&WaitForTheGoAhead::pauseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "[WaitForTheGoAhead] Initialized");
    }

    BT::NodeStatus WaitForTheGoAhead::tick()
    {
        // 1) Controllo sicurezza area
        if (!area_is_currently_safe_.load(std::memory_order_relaxed))
        {
            if (!first_tick_unsafe_logged_)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[WaitForTheGoAhead] Area unsafe, pausing until safe");
                first_tick_unsafe_logged_ = true;
            }
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                5000,
                "[WaitForTheGoAhead] Waiting for area to become safe");
            return BT::NodeStatus::RUNNING;
        }
        else if (first_tick_unsafe_logged_)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "[WaitForTheGoAhead] Area is now safe");
            first_tick_unsafe_logged_ = false;
        }

        // 2) Controllo richiesta resume
        if (!human_resume_requested_.load(std::memory_order_relaxed))
        {
            if (!first_tick_pause_logged_)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[WaitForTheGoAhead] Human requested pause, waiting for resume");
                first_tick_pause_logged_ = true;
            }
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                5000,
                "[WaitForTheGoAhead] Waiting for human resume");
            return BT::NodeStatus::RUNNING;
        }
        else if (first_tick_pause_logged_)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "[WaitForTheGoAhead] Human resumed, proceeding");
            first_tick_pause_logged_ = false;
        }

        // 3) Tutto ok: SUCCESS
        return BT::NodeStatus::SUCCESS;
    }

    void WaitForTheGoAhead::halt()
    {
        RCLCPP_INFO(node_->get_logger(), "[WaitForTheGoAhead] Halted");
        first_tick_unsafe_logged_ = false;
        first_tick_pause_logged_ = false;
        CoroActionNode::halt();
    }

    void WaitForTheGoAhead::safetyCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // msg->data == true  → umano vicino  → area non sicura
        bool new_safe = !msg->data;
        area_is_currently_safe_.store(new_safe, std::memory_order_relaxed);
    }

    void WaitForTheGoAhead::pauseCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // msg->data == true  → umano chiede pausa  → resume_requested = false
        bool new_resume = !msg->data;
        human_resume_requested_.store(new_resume, std::memory_order_relaxed);
    }

} // namespace cr::bt::orchestrator::nodes
