#include "cr_bt_orchestrator/custom_bt_nodes.hpp"

namespace cr
{
	namespace bt_nodes
	{

		bool GetObjectInfo::setRequest(typename cr_interfaces::srv::GetObjectInfo::Request::SharedPtr & request)
		{
			// Recupero di object_id come stringa dalla blackboard
			std::string object_id_str;
			if (!getInput("object_id", object_id_str)) {
			  RCLCPP_ERROR(logger(), "Missing input [object_id]");
			  return false;
			}

			// Conversione object_id da stringa a uint8_t, con gestione errori
			try {
			  int tmp = std::stoi(object_id_str);
			  if (tmp < 0 || tmp > std::numeric_limits<uint8_t>::max()) {
				RCLCPP_ERROR(logger(),
							 "object_id out of range [0..%u]", 
							 static_cast<unsigned>(std::numeric_limits<uint8_t>::max()));
				return false;
			  }
			  request->id = static_cast<uint8_t>(tmp);
			} catch (const std::exception & e) {
			  RCLCPP_ERROR(logger(),
						   "Invalid object_id '%s': %s",
						   object_id_str.c_str(), e.what());
			  return false;
			}
		  
			return true;
		}  

		BT::NodeStatus GetObjectInfo::onResponseReceived(const typename cr_interfaces::srv::GetObjectInfo::Response::SharedPtr &response)
		{
			if (!response) {
				RCLCPP_ERROR(logger(), "Service call failed (empty response)");
				return BT::NodeStatus::FAILURE;
			}
		
			if (!response->success) {
				RCLCPP_ERROR(logger(), "Service returned failure");
				return BT::NodeStatus::FAILURE;
			}
		
			// Impostazione porta di output "object_info" con dati ricevuti
			setOutput("object_info", response->object_info);
		
			RCLCPP_INFO(logger(), "Received object info: center=[%.2f, %.2f, %.2f]",
						response->object_info.center.x, response->object_info.center.y,
						response->object_info.center.z);
		
			return BT::NodeStatus::SUCCESS;
		}

		BT::NodeStatus ExecutePick::tick()
		{
			auto object_info = getInput<cr_interfaces::msg::ObjectInfo>("object_info");

			if (!object_info)
			{
				RCLCPP_ERROR(rclcpp::get_logger("ExecutePickNode"),
							 "Missing required input ports for ExecutePick");
				return BT::NodeStatus::FAILURE;
			}

			RCLCPP_INFO(rclcpp::get_logger("ExecutePickNode"),
						"Simulating ExecutePick for object");

			RCLCPP_INFO(rclcpp::get_logger("ExecutePickNode"),
						"Pick successful (simulated).");
			return BT::NodeStatus::SUCCESS;
		}

		BT::NodeStatus ExecutePlace::tick()
		{
			auto object_info = getInput<cr_interfaces::msg::ObjectInfo>("object_info");

			if (!object_info)
			{
				RCLCPP_ERROR(rclcpp::get_logger("ExecutePlaceNode"),
							 "Missing required input ports for ExecutePlace");
				return BT::NodeStatus::FAILURE;
			}

			RCLCPP_INFO(rclcpp::get_logger("ExecutePlaceNode"),
						"Simulating ExecutePlace for object");

			RCLCPP_INFO(rclcpp::get_logger("ExecutePlaceNode"),
						"Place successful (simulated).");
			return BT::NodeStatus::SUCCESS;
		}

		BT::NodeStatus CheckHumanPresence::tick()
		{
			bool human_present = true; // Simulazione della presenza umana
			RCLCPP_INFO(rclcpp::get_logger("CheckHumanNode"), "human_detected = %s", human_present ? "true" : "false");
			if (human_present)
			{
				RCLCPP_INFO(rclcpp::get_logger("CheckHumanPresenceNode"),
							"Human presence detected.");
				return BT::NodeStatus::FAILURE;
			}
			else
			{
				RCLCPP_INFO(rclcpp::get_logger("CheckHumanPresenceNode"),
							"No human presence detected.");
				return BT::NodeStatus::SUCCESS;
			}
		}

		BT::NodeStatus PauseRobot::tick()
		{
			RCLCPP_INFO(rclcpp::get_logger("PauseRobot"),
						"Simulating StopRobot action");
			return BT::NodeStatus::SUCCESS;
		}
	
	
	} // namespace bt_nodes
} // namespace cr
