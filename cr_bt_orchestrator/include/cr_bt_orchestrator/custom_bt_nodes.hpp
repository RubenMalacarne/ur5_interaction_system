#ifndef CR_BT_ORCHESTRATOR__CUSTOM_BT_NODES_HPP_
#define CR_BT_ORCHESTRATOR__CUSTOM_BT_NODES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <cr_interfaces/msg/object_info.hpp>

namespace cr
{
  namespace bt_nodes
  {

    // Nodo dedicato al recupero delle informazioni di un oggetto.
    // Interroga cr_vision
    class GetObjectInfo : public BT::SyncActionNode
    {
    public:
      GetObjectInfo(const std::string &name, const BT::NodeConfiguration &config)
          : BT::SyncActionNode(name, config) {}

      static BT::PortsList providedPorts()
      {
        return {BT::InputPort<std::string>("object_id"),
                BT::OutputPort<cr_interfaces::msg::ObjectInfo>("object_info")};
      }

      BT::NodeStatus tick() override;
    };

    // Nodo dedicato all'azione di Pick - chiama l'action server
    class ExecutePick : public BT::SyncActionNode
    {
    public:
      ExecutePick(const std::string &name, const BT::NodeConfiguration &config)
          : BT::SyncActionNode(name, config) {}

      static BT::PortsList providedPorts()
      {
        return {BT::InputPort<cr_interfaces::msg::ObjectInfo>("object_info")};
      }

      BT::NodeStatus tick() override;
    };

    // Nodo dedicato all'azione di Place - chiama l'action server
    class ExecutePlace : public BT::SyncActionNode
    {
    public:
      ExecutePlace(const std::string &name, const BT::NodeConfiguration &config)
          : BT::SyncActionNode(name, config) {}

      static BT::PortsList providedPorts()
      {
        return {BT::InputPort<cr_interfaces::msg::ObjectInfo>("object_info")};
      }

      BT::NodeStatus tick() override;
    };

    // Nodo dedicato al logging
    class LogMessage : public BT::SyncActionNode
    {
    public:
      LogMessage(const std::string &name, const BT::NodeConfiguration &config)
          : BT::SyncActionNode(name, config) {}

      static BT::PortsList providedPorts()
      {
        return {BT::InputPort<std::string>("message")};
      }

      BT::NodeStatus tick() override
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
    };

  } // namespace bt_nodes
} // namespace cr

#endif // CR_BT_ORCHESTRATOR__CUSTOM_BT_NODES_HPP_
