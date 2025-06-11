#include "cr_bt_common/gui_log_node.hpp"

namespace cr::bt::common
{

    GuiLog::GuiLog(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config)
    {
        // Recupera il nodo ROS dalla blackboard
        auto blackboard = config.blackboard;
        auto node = blackboard->get<rclcpp::Node::SharedPtr>("ros_node");

        auto qos = rclcpp::QoS(10).transient_local();
        static auto shared_pub = node->create_publisher<cr_interfaces::msg::Log>("cr/gui_log", qos);
        pub_ = shared_pub;
    }

    BT::PortsList GuiLog::providedPorts()
    {
        return {
            BT::InputPort<std::string>("main_msg", "Main message (required)"),
            BT::InputPort<std::string>("log_msg", "Log message (required)"),
            BT::InputPort<std::string>("phase", "Phase (optional)"),
            BT::InputPort<uint8_t>("percentage", "Progress % (optional)"),
            BT::InputPort<int8_t>("target_id", "Target ID (optional)"),
            BT::InputPort<std::string>("target_label", "Target Label (optional)")};
    }

    BT::NodeStatus GuiLog::tick()
    {
        cr_interfaces::msg::Log m;
        m.target_id = -1; // default “none”
        m.severity = 0;   // default INFO

        // richiesti
        if (!getInput("main_msg", m.main_msg))
            throw BT::RuntimeError("GuiLog: missing port [main_msg]");
        if (!getInput("log_msg", m.log_msg))
            throw BT::RuntimeError("GuiLog: missing port [log_msg]");

        // phase + percentage insieme
        {
            std::string phase;
            uint8_t pct = 0;
            if (getInput("phase", phase) && getInput("percentage", pct))
            {
                m.phase = phase;
                m.percentage = pct;
            }
        }

        // target_id opzionale
        {
            int8_t tid = -1;
            if (getInput("target_id", tid))
            {
                m.target_id = tid;
            }
        }

        {
            std::string target_label;
            if (getInput("target_label", target_label))
            {
                m.target_label = target_label;
            }
        }

        pub_->publish(m);
        return BT::NodeStatus::SUCCESS;
    }

} // namespace cr::bt::common
