#ifndef CR_BT_ORCHESTRATOR__CUSTOM_BT_NODES_HPP_
#define CR_BT_ORCHESTRATOR__CUSTOM_BT_NODES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <cr_interfaces/msg/object_info.hpp>
#include <cr_interfaces/srv/get_object_info.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>

namespace cr
{
	namespace bt_nodes
	{
		// Service Wrapper
		class GetObjectInfo : public BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>
		{
		public:
			GetObjectInfo(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
				: BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>(instance_name, conf, params) {}

			static BT::PortsList providedPorts()
			{
				return BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>::providedBasicPorts({
					BT::InputPort<std::string>("object_id"),
					BT::OutputPort<cr_interfaces::msg::ObjectInfo>("object_info")
				});
			}

			bool setRequest(typename cr_interfaces::srv::GetObjectInfo::Request::SharedPtr &request) override;
			BT::NodeStatus onResponseReceived(const typename cr_interfaces::srv::GetObjectInfo::Response::SharedPtr &response) override;
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
