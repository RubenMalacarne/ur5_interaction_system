#ifndef COPPELIA_SIM_HARDWARE_INTERFACE_HPP
#define COPPELIA_SIM_HARDWARE_INTERFACE_HPP

/**
 * @file coppelia_sim_hardware_interface.hpp
 * @brief Hardware Interface of ROS2 for controlling and monitoring a robot in CoppeliaSim.
 */

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/sensor_msgs/msg/joint_state.hpp"

namespace cr_hw_configuration {


    /**
     * @class CoppeliaSimHardwareInterface
     * @brief Implements SystemInterface for integrating CoppeliaSim with ROS 2 control.
    */

    class CoppeliaSimHardwareInterface : public hardware_interface::SystemInterface
    {
        public:
            CoppeliaSimHardwareInterface();
            virtual ~CoppeliaSimHardwareInterface();

            // LifecycleNodeInterface Methods
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

            // SystemInterface Methods - Override
            /** Inizialaize hardware interface with dates of the robot from URDF file */
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            
            /** Export state interface for the control system */
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            /** Export command interface for the system control */
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            /** upgrade the current state of hardware, reading the information of the sensors and joints */
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            /** Send updated commands to the physical device */
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            // Callback for joint state subscription
            void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
            void robotiqStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

            
            std::vector<std::string> joint_names_ = {
                "shoulder_pan_joint", 
                "shoulder_lift_joint", 
                "elbow_joint",
                "wrist_1_joint", 
                "wrist_2_joint", 
                "wrist_3_joint"
            };

            std::vector<std::string> gripper_joint_names_ = {
                "robotiq_85_left_knuckle_joint",
                "robotiq_85_right_knuckle_joint",
                "robotiq_85_left_inner_knuckle_joint",
                "robotiq_85_right_inner_knuckle_joint",
                "robotiq_85_left_finger_tip_joint",
                "robotiq_85_right_finger_tip_joint"
            };

            // Vectors to store the current state of the joints
            std::vector<double> joint_positions_;
            std::vector<double> joint_velocities_;
            std::vector<double> joint_efforts_;

            std::vector<double> joint_positions_robotiq_;
            std::vector<double> joint_velocities_robotiq_;

            // These vectors will be filled with the commands to be sent to the joints
            std::vector<double> joint_commands_position_;
            std::vector<double> joint_commands_velocity_;
            std::vector<double> joint_commands_position_robotiq_;
            std::vector<double> joint_commands_velocity_robotiq_;
            
            sensor_msgs::msg::JointState::SharedPtr last_joint_state_robotiq_;
            sensor_msgs::msg::JointState::SharedPtr last_joint_state_;

            bool is_active_{false};
            std::vector<double> joint_commands_;
            rclcpp::Node::SharedPtr node_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_robotiq_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_robotiq_;

    };

}

#endif
// NOTE:
// per publicare il movimento del gripper: 
// ros2 topic pub /gripper_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
//     joint_names: ['robotiq_joint_0', 'robotiq_joint_1', 'robotiq_joint_2', 'robotiq_joint_3', 'robotiq_joint_4', 'robotiq_joint_5'],
//     points: [{
//       positions: [0.50, -0.50, 0.50, -0.50, 0.5, -0.50],
//       time_from_start: {sec: 2, nanosec: 0}
//     }]
//   }"