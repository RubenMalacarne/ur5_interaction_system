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
        static auto shared_pub = node->create_publisher<std_msgs::msg::String>("cr/gui_log", qos);
        pub_ = shared_pub;
    }

    BT::PortsList GuiLog::providedPorts()
    {
        return {BT::InputPort<std::string>("text", "Messaggio da mandare alla GUI")};
    }

    BT::NodeStatus GuiLog::tick()
    {
        std::string txt;
        getInput("text", txt);

        std_msgs::msg::String m;
        m.data = txt;
        pub_->publish(m);
        return BT::NodeStatus::SUCCESS;
    }

} // namespace cr::bt::common
