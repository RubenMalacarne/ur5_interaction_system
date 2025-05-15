#ifndef CR_BT_ORCHESTRATOR__CUSTOM_BT_NODES_HPP_
#define CR_BT_ORCHESTRATOR__CUSTOM_BT_NODES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <cr_interfaces/msg/object_info.hpp>
#include <cr_interfaces/srv/get_object_info.hpp>
#include <cr_interfaces/srv/freeze_scene.hpp>
#include <cr_interfaces/action/pick.hpp>
#include <cr_interfaces/action/place.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>

namespace cr
{
	namespace bt_nodes
	{
		#define BT_ACTION_LOG_INFO(fmt, ...)  RCLCPP_INFO(logger(),  "[%s] " fmt, this->name().c_str(), ##__VA_ARGS__)
		#define BT_ACTION_LOG_WARN(fmt, ...)  RCLCPP_WARN(logger(),  "[%s] " fmt, this->name().c_str(), ##__VA_ARGS__)
    	#define BT_ACTION_LOG_ERROR(fmt, ...) RCLCPP_ERROR(logger(), "[%s] " fmt, this->name().c_str(), ##__VA_ARGS__)

		// ------------------------------------------------------------------------------------------------------------------
		//                                       Service Wrapper - GET OBJECT INFO
		// ------------------------------------------------------------------------------------------------------------------
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


		// ------------------------------------------------------------------------------------------------------------------
		//                                       Service Wrapper - FREEZE SCENE
		// ------------------------------------------------------------------------------------------------------------------
		class FreezeScene : public BT::RosServiceNode<cr_interfaces::srv::FreezeScene>
		{
		public:
			FreezeScene(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
				: BT::RosServiceNode<cr_interfaces::srv::FreezeScene>(instance_name, conf, params) {}

			bool setRequest(typename Request::SharedPtr& request) override;
			BT::NodeStatus onResponseReceived(const typename Response::SharedPtr &response) override;
		};

		// ------------------------------------------------------------------------------------------------------------------
		//                                       Action Wrapper - EXECUTE PICK
		// ------------------------------------------------------------------------------------------------------------------
		class ExecutePick : public BT::RosActionNode<cr_interfaces::action::Pick>
		{
		public:
			ExecutePick(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
				: BT::RosActionNode<cr_interfaces::action::Pick>(instance_name, conf, params) {}

			static BT::PortsList providedPorts();

			bool setGoal(Goal & goal) override;
			BT::NodeStatus onResultReceived(const WrappedResult& result) override;
			BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
			BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
		};


		// ------------------------------------------------------------------------------------------------------------------
		//                                       Action Wrapper - EXECUTE PLACE
		// ------------------------------------------------------------------------------------------------------------------
		class ExecutePlace : public BT::RosActionNode<cr_interfaces::action::Place>
		{
		public:
			ExecutePlace(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
				: BT::RosActionNode<cr_interfaces::action::Place>(instance_name, conf, params) {}

			static BT::PortsList providedPorts();

			bool setGoal(Goal & goal) override;
			BT::NodeStatus onResultReceived(const WrappedResult& result) override;
			BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
			BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
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
