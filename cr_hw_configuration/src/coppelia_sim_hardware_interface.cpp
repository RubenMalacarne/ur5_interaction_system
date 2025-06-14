#include "cr_hw_configuration/coppelia_sim_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace cr::hw_configuration
{

	CoppeliaSimHardwareInterface::CoppeliaSimHardwareInterface()
		: joint_positions_(6, 0.0),
		  joint_velocities_(6, 0.0),
		  joint_efforts_(6, 0.0),
		  joint_positions_robotiq_(6, 0.0),
		  joint_velocities_robotiq_(6, 0.0),
		  joint_commands_position_(6, 0.0),
		  joint_commands_velocity_(6, 0.0),
		  joint_commands_position_robotiq_(6, 0.0),
		  joint_commands_velocity_robotiq_(6, 0.0)
	{
	}

	CoppeliaSimHardwareInterface::~CoppeliaSimHardwareInterface() {}

	hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
	{
		if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
			return hardware_interface::CallbackReturn::ERROR;

		RCLCPP_INFO(rclcpp::get_logger("CoppeliaSimHardwareInterface"), "on_init completed.");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
	{
		if (!node_)
			node_ = rclcpp::Node::make_shared("coppelia_sim_hardware_interface_node");

		publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/coppelia_set_joints", 10);
		subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
			"/coppelia_joint_states", 10,
			std::bind(&CoppeliaSimHardwareInterface::jointStateCallback, this, std::placeholders::_1));

		publisher_robotiq_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/coppelia_set", 10);
		subscriber_robotiq_ = node_->create_subscription<sensor_msgs::msg::JointState>(
			"/coppelia_joint", 10,
			std::bind(&CoppeliaSimHardwareInterface::robotiqStateCallback, this, std::placeholders::_1));

		RCLCPP_INFO(node_->get_logger(), "on_configure completed.");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	void CoppeliaSimHardwareInterface::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		last_joint_state_ = msg;
	}

	void CoppeliaSimHardwareInterface::robotiqStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		last_joint_state_robotiq_ = msg;
	}

	hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
	{
		auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("CoppeliaSimHardwareInterface");

		publisher_.reset();
		subscriber_.reset();
		publisher_robotiq_.reset();
		subscriber_robotiq_.reset();
		node_.reset();

		RCLCPP_INFO(logger, "on_cleanup completed.");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
	{
		is_active_ = true;

		joint_commands_position_ = {1.57, -1.57, 1.57, -1.57, 1.57, 0.0};
		joint_commands_velocity_ = std::vector<double>(6, 0.0);
		joint_commands_position_robotiq_ = std::vector<double>(6, 0.0);
		joint_commands_velocity_robotiq_ = std::vector<double>(6, 0.0);

		RCLCPP_INFO(node_->get_logger(), "on_activate: hardware ready.");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
	{
		is_active_ = false;

		for (auto &cmd : joint_commands_position_)
			cmd = 0.0;
		for (auto &cmd : joint_commands_velocity_)
			cmd = 0.0;

		RCLCPP_INFO(node_->get_logger(), "on_deactivate: hardware deactivated.");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
	{
		is_active_ = false;

		for (auto &cmd : joint_commands_position_)
			cmd = 0.0;
		for (auto &cmd : joint_commands_velocity_)
			cmd = 0.0;

		RCLCPP_INFO(node_->get_logger(), "on_shutdown: hardware shutdown.");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_error(const rclcpp_lifecycle::State &)
	{
		is_active_ = false;
		RCLCPP_ERROR(rclcpp::get_logger("CoppeliaSimHardwareInterface"), "Hardware interface error.");
		return hardware_interface::CallbackReturn::SUCCESS;
	}

	std::vector<hardware_interface::StateInterface> CoppeliaSimHardwareInterface::export_state_interfaces()
	{
		std::vector<hardware_interface::StateInterface> state_interfaces;

		for (size_t i = 0; i < joint_names_.size(); ++i)
		{
			state_interfaces.emplace_back(joint_names_[i], "position", &joint_positions_[i]);
			state_interfaces.emplace_back(joint_names_[i], "velocity", &joint_velocities_[i]);
			state_interfaces.emplace_back(joint_names_[i], "effort", &joint_efforts_[i]);
		}

		for (size_t i = 0; i < gripper_joint_names_.size(); ++i)
		{
			state_interfaces.emplace_back(gripper_joint_names_[i], "position", &joint_positions_robotiq_[i]);
			state_interfaces.emplace_back(gripper_joint_names_[i], "velocity", &joint_velocities_robotiq_[i]);
		}

		return state_interfaces;
	}

	std::vector<hardware_interface::CommandInterface> CoppeliaSimHardwareInterface::export_command_interfaces()
	{
		std::vector<hardware_interface::CommandInterface> command_interfaces;

		for (size_t i = 0; i < joint_names_.size(); ++i)
		{
			command_interfaces.emplace_back(joint_names_[i], "position", &joint_commands_position_[i]);
			command_interfaces.emplace_back(joint_names_[i], "velocity", &joint_commands_velocity_[i]);
		}

		for (size_t i = 0; i < gripper_joint_names_.size(); ++i)
		{
			command_interfaces.emplace_back(gripper_joint_names_[i], "position", &joint_commands_position_robotiq_[i]);
		}

		return command_interfaces;
	}

	hardware_interface::return_type CoppeliaSimHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
	{
		rclcpp::spin_some(node_);

		if (last_joint_state_)
		{
			for (size_t i = 0; i < joint_names_.size(); ++i)
			{
				if (i < last_joint_state_->position.size())
					joint_positions_[i] = last_joint_state_->position[i];
				if (i < last_joint_state_->velocity.size())
					joint_velocities_[i] = last_joint_state_->velocity[i];
				if (i < last_joint_state_->effort.size())
					joint_efforts_[i] = last_joint_state_->effort[i];
			}
		}

		if (last_joint_state_robotiq_)
		{
			for (size_t i = 0; i < joint_positions_robotiq_.size(); ++i)
			{
				if (i < last_joint_state_robotiq_->position.size())
					joint_positions_robotiq_[i] = last_joint_state_robotiq_->position[i];
				if (i < last_joint_state_robotiq_->velocity.size())
					joint_velocities_robotiq_[i] = last_joint_state_robotiq_->velocity[i];
				if (i < last_joint_state_robotiq_->effort.size())
					joint_efforts_[i] = last_joint_state_robotiq_->effort[i];
				else
					joint_efforts_[i] = 0.0;
			}
		}

		return hardware_interface::return_type::OK;
	}

	hardware_interface::return_type CoppeliaSimHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
	{
		std_msgs::msg::Float64MultiArray command_msg;
		command_msg.data = joint_commands_position_;

		if (publisher_)
			publisher_->publish(command_msg);
		else
			return hardware_interface::return_type::ERROR;

		joint_commands_position_robotiq_[1] = -joint_commands_position_robotiq_[0];
		joint_commands_position_robotiq_[2] = joint_commands_position_robotiq_[0];
		joint_commands_position_robotiq_[3] = -joint_commands_position_robotiq_[0];
		joint_commands_position_robotiq_[4] = -joint_commands_position_robotiq_[0];
		joint_commands_position_robotiq_[5] = joint_commands_position_robotiq_[0];

		std_msgs::msg::Float64MultiArray command_msg_robotiq;
		command_msg_robotiq.data = joint_commands_position_robotiq_;

		if (publisher_robotiq_)
			publisher_robotiq_->publish(command_msg_robotiq);
		else
			return hardware_interface::return_type::ERROR;

		return hardware_interface::return_type::OK;
	}

} // namespace cr::hw_configuration

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cr::hw_configuration::CoppeliaSimHardwareInterface, hardware_interface::SystemInterface)
