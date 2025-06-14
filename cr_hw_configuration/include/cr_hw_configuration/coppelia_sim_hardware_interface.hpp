/**
 * @file coppelia_sim_hardware_interface.hpp
 * @brief Hardware interface plugin for controlling UR5 and Robotiq gripper in CoppeliaSim via ROS 2.
 *
 * This class implements the SystemInterface for integrating the simulated UR5 robot and Robotiq gripper
 * in CoppeliaSim with `ros2_control`. It allows bidirectional communication with simulated joints using standard ROS messages.
 */

#ifndef CR_HW_CONFIGURATION__COPPELIA_SIM_HARDWARE_INTERFACE_HPP_
#define CR_HW_CONFIGURATION__COPPELIA_SIM_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace cr::hw_configuration
{

    /**
     * @class CoppeliaSimHardwareInterface
     * @brief ROS 2 hardware interface for CoppeliaSim to control UR5 and Robotiq gripper.
     *
     * This class exposes state and command interfaces for a simulated robot
     * in CoppeliaSim, handling lifecycle transitions and communication over ROS topics.
     */
    class CoppeliaSimHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        CoppeliaSimHardwareInterface();
        virtual ~CoppeliaSimHardwareInterface();

        /// Lifecycle transition: configure resources and ROS communication.
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        /// Lifecycle transition: cleanup ROS interfaces and internal state.
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        /// Lifecycle transition: handle proper shutdown of the interface.
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        /// Lifecycle transition: activate the interface (start control).
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        /// Lifecycle transition: deactivate the interface (stop control).
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        /// Lifecycle transition: handle error state.
        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

        /// Initialize the hardware interface using the URDF configuration.
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        /// Export the state interfaces (position, velocity, effort) of all joints.
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /// Export the command interfaces (position, velocity) of all joints.
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /// Read joint states from subscribed ROS topics and update local buffers.
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /// Publish joint commands to ROS topics for CoppeliaSim.
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        /// Callback for receiving joint states of the UR5 arm.
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

        /// Callback for receiving joint states of the Robotiq gripper.
        void robotiqStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

        /// Names of UR5 joints.
        std::vector<std::string> joint_names_ = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"};

        /// Names of Robotiq gripper joints.
        std::vector<std::string> gripper_joint_names_ = {
            "robotiq_85_left_knuckle_joint",
            "robotiq_85_right_knuckle_joint",
            "robotiq_85_left_inner_knuckle_joint",
            "robotiq_85_right_inner_knuckle_joint",
            "robotiq_85_left_finger_tip_joint",
            "robotiq_85_right_finger_tip_joint"};

        /// Joint states for UR5: position, velocity, effort.
        std::vector<double> joint_positions_;
        std::vector<double> joint_velocities_;
        std::vector<double> joint_efforts_;

        /// Joint states for Robotiq: position, velocity.
        std::vector<double> joint_positions_robotiq_;
        std::vector<double> joint_velocities_robotiq_;

        /// Command buffers for UR5 joints.
        std::vector<double> joint_commands_position_;
        std::vector<double> joint_commands_velocity_;

        /// Command buffers for Robotiq joints.
        std::vector<double> joint_commands_position_robotiq_;
        std::vector<double> joint_commands_velocity_robotiq_;

        /// Last received joint states.
        sensor_msgs::msg::JointState::SharedPtr last_joint_state_robotiq_;
        sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

        /// Flag to indicate active control.
        bool is_active_{false};

        /// Optional unified command buffer (not actively used).
        std::vector<double> joint_commands_;

        /// Node used for ROS communication.
        rclcpp::Node::SharedPtr node_;

        /// Publisher for UR5 joint commands.
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

        /// Subscriber for UR5 joint states.
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;

        /// Publisher for Robotiq joint commands.
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_robotiq_;

        /// Subscriber for Robotiq joint states.
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_robotiq_;
    };

} // namespace cr::hw_configuration

#endif // CR_HW_CONFIGURATION__COPPELIA_SIM_HARDWARE_INTERFACE_HPP_
