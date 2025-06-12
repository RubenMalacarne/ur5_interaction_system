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

        setOutput("object_id", std::to_string(response->object_info.id));
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

        gui_log_pub_ = config.blackboard->get<rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr>("gui_log_pub");
        if (!gui_log_pub_)
        {
            throw BT::RuntimeError("Missing gui_log_pub in blackboard");
        }
    }

    BT::PortsList IsAreaSafe::providedPorts()
    {
        return {};
    }

    BT::NodeStatus IsAreaSafe::tick()
    {
        if (human_near_.load(std::memory_order_relaxed))
        {
            // cr_interfaces::msg::Log log_msg;
            // log_msg.main_msg = "Paused - Area Unsafe";
            // log_msg.log_msg = "Paused - Area Unsafe";
            // log_msg.target_id = -1;
            // log_msg.percentage = 0;
            // gui_log_pub_->publish(log_msg);
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            return BT::NodeStatus::SUCCESS;
        }
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

        gui_log_pub_ = config.blackboard->get<rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr>("gui_log_pub");
        if (!gui_log_pub_)
        {
            throw BT::RuntimeError("Missing gui_log_pub in blackboard");
        }
    }

    BT::PortsList IsStopRequested::providedPorts()
    {
        return {};
    }

    BT::NodeStatus IsStopRequested::tick()
    {
        if (stop_requested_.load(std::memory_order_relaxed))
        {
            cr_interfaces::msg::Log log_msg;
            log_msg.main_msg = "Execution cancellation...";
            log_msg.log_msg = "Cancel request received";
            log_msg.target_id = -1;
            log_msg.percentage = 0;
            gui_log_pub_->publish(log_msg);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
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

        gui_log_pub_ = config.blackboard->get<rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr>("gui_log_pub");
        if (!gui_log_pub_)
        {
            throw BT::RuntimeError("Missing gui_log_pub in blackboard");
        }
    }

    BT::PortsList IsPauseRequested::providedPorts()
    {
        return {};
    }

    BT::NodeStatus IsPauseRequested::tick()
    {
        if (pause_requested_.load(std::memory_order_relaxed))
        {
            // cr_interfaces::msg::Log log_msg;
            // log_msg.main_msg = "Pause";
            // log_msg.log_msg = "Pause";
            // log_msg.target_id = -1;
            // log_msg.percentage = 0;
            // gui_log_pub_->publish(log_msg);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
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

        gui_log_pub_ = config.blackboard->get<rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr>("gui_log_pub");
        if (!gui_log_pub_)
        {
            throw BT::RuntimeError("Missing gui_log_pub in blackboard");
        }
        RCLCPP_INFO(node_->get_logger(), "[WaitForTheGoAhead] Initialized");
    }

    BT::NodeStatus WaitForTheGoAhead::tick()
    {
        // 1) Controllo sicurezza area
        if (!area_is_currently_safe_.load(std::memory_order_relaxed))
        {
            if (!first_tick_unsafe_logged_)
            {
                cr_interfaces::msg::Log log_msg;
                log_msg.main_msg = "Paused - Area Unsafe";
                log_msg.log_msg = "Paused - Area Unsafe";
                log_msg.target_id = -1;
                log_msg.percentage = 0;
                gui_log_pub_->publish(log_msg);
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
            cr_interfaces::msg::Log log_msg;

            if(first_tick_pause_logged_){
                log_msg.main_msg = "Pause";
                log_msg.log_msg = "Area is safe, but the execution is paused";
            } else {
                log_msg.main_msg = "Executing Workflow";
                log_msg.log_msg = "Area is safe";
            }


            log_msg.target_id = -1;
            log_msg.percentage = 0;
            gui_log_pub_->publish(log_msg);
            RCLCPP_INFO(node_->get_logger(),
                        "[WaitForTheGoAhead] Area is now safe");
            first_tick_unsafe_logged_ = false;
        }

        // 2) Controllo richiesta resume
        if (!human_resume_requested_.load(std::memory_order_relaxed))
        {
            if (!first_tick_pause_logged_)
            {
                cr_interfaces::msg::Log log_msg;
                log_msg.main_msg = "Pause";
                log_msg.log_msg = "Pause";
                log_msg.target_id = -1;
                log_msg.percentage = 0;
                gui_log_pub_->publish(log_msg);
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
            cr_interfaces::msg::Log log_msg;
            if(first_tick_unsafe_logged_){
                log_msg.main_msg = "Pause - Area Unsafe";
                log_msg.log_msg = "Request to resume received, but area is unsafe";
            } else {
                log_msg.main_msg = "Executing Workflow";
                log_msg.log_msg = "Request to resume received";
            }
            log_msg.target_id = -1;
            log_msg.percentage = 0;
            gui_log_pub_->publish(log_msg);
            RCLCPP_INFO(node_->get_logger(),
                        "[WaitForTheGoAhead] Human resumed, proceeding");

            first_tick_pause_logged_ = false;
        }

        cr_interfaces::msg::Log log_msg;
        log_msg.main_msg = "Executing Workflow";
        log_msg.log_msg = "Resume";
        log_msg.target_id = -1;
        log_msg.percentage = 0;
        gui_log_pub_->publish(log_msg);
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
