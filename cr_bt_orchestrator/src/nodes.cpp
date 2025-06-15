#include "cr_bt_orchestrator/nodes.hpp"

#include <functional>
#include <limits>
#include <stdexcept>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace cr::bt::orchestrator::nodes
{

    // -------- GetObjectInfo --------

    GetObjectInfo::GetObjectInfo(const std::string &instance_name,
                                 const BT::NodeConfig &conf,
                                 const BT::RosNodeParams &params)
        : BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>(instance_name, conf, params)
    {
    }

    BT::PortsList GetObjectInfo::providedPorts()
    {
        return BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>::providedBasicPorts({BT::InputPort<std::string>("object_label"),
                                                                                          BT::OutputPort<std::string>("object_id"),
                                                                                          BT::OutputPort<double>("object_center_x"),
                                                                                          BT::OutputPort<double>("object_center_y"),
                                                                                          BT::OutputPort<double>("object_center_z"),
                                                                                          BT::OutputPort<double>("object_size_x"),
                                                                                          BT::OutputPort<double>("object_size_y"),
                                                                                          BT::OutputPort<double>("object_size_z")});
    }

    bool GetObjectInfo::setRequest(typename Request::SharedPtr &request)
    {
        std::string object_label;
        if (!getInput("object_label", object_label))
        {
            BT_ACTION_LOG_ERROR("Missing input [object_label]");
            return false;
        }
        request->label = object_label;
        return true;
    }

    BT::NodeStatus GetObjectInfo::onResponseReceived(const typename Response::SharedPtr &response)
    {
        if (!response || !response->success)
        {
            BT_ACTION_LOG_ERROR("Service returned failure or empty response");
            return BT::NodeStatus::FAILURE;
        }

        // Set outputs from response
        setOutput("object_id", std::to_string(response->object_info.id));
        setOutput("object_center_x", response->object_info.center.x);
        setOutput("object_center_y", response->object_info.center.y);
        setOutput("object_center_z", response->object_info.center.z);
        setOutput("object_size_x", response->object_info.size.x);
        setOutput("object_size_y", response->object_info.size.y);
        setOutput("object_size_z", response->object_info.size.z);

        BT_ACTION_LOG_INFO("Received object info: center=[%.2f, %.2f, %.2f]",
                           response->object_info.center.x,
                           response->object_info.center.y,
                           response->object_info.center.z);
        return BT::NodeStatus::SUCCESS;
    }

    // -------- FreezeScene --------

    FreezeScene::FreezeScene(const std::string &instance_name,
                             const BT::NodeConfig &conf,
                             const BT::RosNodeParams &params)
        : BT::RosServiceNode<cr_interfaces::srv::FreezeScene>(instance_name, conf, params)
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
        if (!response || !response->success)
        {
            BT_ACTION_LOG_ERROR("Service returned failure or empty response");
            return BT::NodeStatus::FAILURE;
        }

        BT_ACTION_LOG_INFO("Planning scene successfully frozen");
        return BT::NodeStatus::SUCCESS;
    }

    // -------- IsAreaSafe --------

    IsAreaSafe::IsAreaSafe(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        auto ros_node = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
        if (!ros_node)
            throw BT::RuntimeError("Missing 'ros_node' in blackboard");

        safety_subscription_ = ros_node->create_subscription<std_msgs::msg::Bool>(
            "/cr/human_near", rclcpp::QoS(10),
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                human_near_.store(msg->data, std::memory_order_relaxed);
            });

        gui_log_pub_ = config.blackboard->get<rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr>("gui_log_pub");
        if (!gui_log_pub_)
            throw BT::RuntimeError("Missing gui_log_pub in blackboard");
    }

    BT::PortsList IsAreaSafe::providedPorts() { return {}; }

    BT::NodeStatus IsAreaSafe::tick()
    {
        return human_near_.load(std::memory_order_relaxed) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

    void IsAreaSafe::updateSafetyStatus(bool is_safe)
    {
        human_near_.store(!is_safe, std::memory_order_relaxed);
    }

    // -------- IsStopRequested --------

    IsStopRequested::IsStopRequested(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        auto ros_node = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
        if (!ros_node)
            throw BT::RuntimeError("Missing 'ros_node' in blackboard");

        stop_command_sub_ = ros_node->create_subscription<std_msgs::msg::Bool>(
            "/cr/stop_command", rclcpp::QoS(10),
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                stop_requested_.store(msg->data, std::memory_order_relaxed);
            });

        gui_log_pub_ = config.blackboard->get<rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr>("gui_log_pub");
        if (!gui_log_pub_)
            throw BT::RuntimeError("Missing gui_log_pub in blackboard");
    }

    BT::PortsList IsStopRequested::providedPorts() { return {}; }

    BT::NodeStatus IsStopRequested::tick()
    {
        if (stop_requested_.load(std::memory_order_relaxed))
        {
            cr_interfaces::msg::Log log_msg;
            log_msg.main_msg = "Execution cancellation...";
            log_msg.log_msg = "Cancel request received";
            log_msg.target_id = -1;
            gui_log_pub_->publish(log_msg);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    void IsStopRequested::updateStopRequestedStatus(bool is_stop_requested)
    {
        stop_requested_.store(is_stop_requested, std::memory_order_relaxed);
    }

    // -------- IsPauseRequested --------

    IsPauseRequested::IsPauseRequested(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        auto ros_node = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
        if (!ros_node)
            throw BT::RuntimeError("Missing 'ros_node' in blackboard");

        pause_command_sub_ = ros_node->create_subscription<std_msgs::msg::Bool>(
            "/cr/pause_command", rclcpp::QoS(10),
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                pause_requested_.store(msg->data, std::memory_order_relaxed);
            });

        gui_log_pub_ = config.blackboard->get<rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr>("gui_log_pub");
        if (!gui_log_pub_)
            throw BT::RuntimeError("Missing gui_log_pub in blackboard");
    }

    BT::PortsList IsPauseRequested::providedPorts() { return {}; }

    BT::NodeStatus IsPauseRequested::tick()
    {
        return pause_requested_.load(std::memory_order_relaxed) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void IsPauseRequested::updatePauseRequestedStatus(bool is_requested)
    {
        pause_requested_.store(is_requested, std::memory_order_relaxed);
    }

    // -------- WaitForTheGoAhead --------

    WaitForTheGoAhead::WaitForTheGoAhead(const std::string &name, const BT::NodeConfig &config)
        : BT::CoroActionNode(name, config)
    {
        node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
        if (!node_)
            throw BT::RuntimeError("Missing 'ros_node' in blackboard");

        safety_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/cr/human_near", rclcpp::QoS(10),
            std::bind(&WaitForTheGoAhead::safetyCallback, this, std::placeholders::_1));

        pause_command_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/cr/pause_command", rclcpp::QoS(10),
            std::bind(&WaitForTheGoAhead::pauseCallback, this, std::placeholders::_1));

        gui_log_pub_ = config.blackboard->get<rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr>("gui_log_pub");
        if (!gui_log_pub_)
            throw BT::RuntimeError("Missing gui_log_pub in blackboard");
    }

    BT::NodeStatus WaitForTheGoAhead::tick()
    {
        if (!area_is_currently_safe_.load())
        {
            if (!first_tick_unsafe_logged_)
            {
                cr_interfaces::msg::Log log_msg;
                log_msg.main_msg = "Paused - Area Unsafe";
                log_msg.log_msg = "Paused - Area Unsafe";
                log_msg.target_id = -1;
                gui_log_pub_->publish(log_msg);
                first_tick_unsafe_logged_ = true;
            }
            return BT::NodeStatus::RUNNING;
        }

        if (!human_resume_requested_.load())
        {
            if (!first_tick_pause_logged_)
            {
                cr_interfaces::msg::Log log_msg;
                log_msg.main_msg = "Pause";
                log_msg.log_msg = "Pause";
                log_msg.target_id = -1;
                gui_log_pub_->publish(log_msg);
                first_tick_pause_logged_ = true;
            }
            return BT::NodeStatus::RUNNING;
        }

        if (first_tick_unsafe_logged_ || first_tick_pause_logged_)
        {
            cr_interfaces::msg::Log log_msg;
            log_msg.main_msg = "Executing Workflow";
            log_msg.log_msg = "Resume";
            log_msg.target_id = -1;
            gui_log_pub_->publish(log_msg);
            first_tick_unsafe_logged_ = false;
            first_tick_pause_logged_ = false;
        }

        return BT::NodeStatus::SUCCESS;
    }

    void WaitForTheGoAhead::halt()
    {
        first_tick_unsafe_logged_ = false;
        first_tick_pause_logged_ = false;
        CoroActionNode::halt();
    }

    void WaitForTheGoAhead::safetyCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        area_is_currently_safe_.store(!msg->data);
    }

    void WaitForTheGoAhead::pauseCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        human_resume_requested_.store(!msg->data);
    }

} // namespace cr::bt::orchestrator::nodes
