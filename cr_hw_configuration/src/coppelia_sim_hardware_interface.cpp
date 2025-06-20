
/**
 * @file coppelia_sim_hardware_interface.cpp
 * @brief Hardware Interface of ROS2 for controlling and monitoring a robot in CoppeliaSim.
 */
#include <memory>
#include <iostream>
#include <vector>
#include <string>

#include "cr_hw_configuration/coppelia_sim_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace cr_hw_configuration
{
  /**
   * @brief constructor: Initialize all buffer of joints to 0
   */
  CoppeliaSimHardwareInterface::CoppeliaSimHardwareInterface() : // UR5
                                                                 joint_positions_(6, 0.0),
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

  CoppeliaSimHardwareInterface::~CoppeliaSimHardwareInterface()
  {
    // Cleanup se necessario
  }

  /**
   * @brief Initialize hardware interface with dates of the robot from URDF file
   * @param info info structure with the info about HW from ros2_control.
   * @return CallbackReturn::SUCCESS o CallbackReturn::ERROR.
   */
  hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (this->SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("CoppeliaSimHardwareInterface"),
        "on_init completato con successo.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief configure the publisher/subscriber ROS2 to interact with CoppeliaSim. 
   * @param previous_state The lifecycle state prior to this transition.
   * @return CallbackReturn::SUCCESS if setup is successful.
  */
  // setup hardware interface
  hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Crea un nodo se non esiste già
    if (!node_)
    {
      node_ = rclcpp::Node::make_shared("coppelia_sim_hardware_interface_node");
    }
    // Topic per UR5
    // coppelia_set_joints --> invia i comandi da coppelia
    publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/coppelia_set_joints", 10);
    // coppelia_joint_states --> riceviamo i comandi da coppelia
    subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/coppelia_joint_states",
        10,
        std::bind(&CoppeliaSimHardwareInterface::jointStateCallback, this, std::placeholders::_1));
    // Topic per il gripper
    publisher_robotiq_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/coppelia_set", 10);

    subscriber_robotiq_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/coppelia_joint",
        10,
        std::bind(&CoppeliaSimHardwareInterface::robotiqStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "on_configure completato: comunicazione configurata correttamente.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Callback for receiving the UR5 joint states from CoppeliaSim.
   * @param msg The received JointState message.
   */
  // void per aggioranre i dati dei giunti di last_joint_state_
  void CoppeliaSimHardwareInterface::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = msg;

    // //debug
    // std::cout << "[CoppeliaSimHardwareInterface] Received JointState con "
    //           << msg->name.size() << " giunture:\n";

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      double pos = (i < msg->position.size()) ? msg->position[i] : 0.0;
      double vel = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
      double eff = (i < msg->effort.size()) ? msg->effort[i] : 0.0;

      //   std::cout << "   [" << i << "] Name: " << msg->name[i]
      //             << " | Pos=" << pos
      //             << ", Vel=" << vel
      //             << ", Eff=" << eff
      //             << "\n";
    }
    // std::cout << std::endl;
  }

  /**
   * @brief Callback for receiving Robotiq gripper joint states from CoppeliaSim.
   * @param msg The received JointState message.
   */
  void CoppeliaSimHardwareInterface::robotiqStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_robotiq_ = msg;
    // //debug
    // std::cout << "[CoppeliaSimHardwareInterface] Received Robotiq JointState with "
    //           << msg->name.size() << " joints:\n";

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      double pos = (i < msg->position.size()) ? msg->position[i] : 0.0;
      double vel = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
      double eff = (i < msg->effort.size()) ? msg->effort[i] : 0.0;

      // std::cout << "   [" << i << "] Name: " << msg->name[i]
      //           << " | Pos=" << pos
      //           << ", Vel=" << vel
      //           << ", Eff=" << eff
      //           << "\n";
    }
    // std::cout << std::endl;
  }


  /**
   * @brief Cleans up resources such as publishers, subscribers, and the ROS node.
   * @return CallbackReturn::SUCCESS after cleanup.
   */
  hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("CoppeliaSimHardwareInterface");

    RCLCPP_INFO(logger, "Cleaning up CoppeliaSimHardwareInterface resources.");

    publisher_.reset();
    subscriber_.reset();

    publisher_robotiq_.reset();
    subscriber_robotiq_.reset();

    node_.reset();

    RCLCPP_INFO(logger, "on_cleanup completato: risorse liberate.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /** Method where hardware 'power' is enable */
  hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    is_active_ = true;

    joint_commands_position_ = {1.57, -1.57, 1.57, -1.57, 1.57, 0.0};
    joint_commands_velocity_ = std::vector<double>(6, 0.0);
    joint_commands_position_robotiq_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    joint_commands_velocity_robotiq_ = std::vector<double>(6, 0.0);
  
    RCLCPP_INFO(node_->get_logger(), "on_activate: hardware attivato con posizione home.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Handles hardware shutdown by zeroing all joint commands and disabling control.
   * @param previous_state The lifecycle state prior to this transition.
   * @return CallbackReturn::SUCCESS.
   */
  hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    is_active_ = false;

    // Azzera i comandi
    for (auto &cmd : joint_commands_position_)
    {
      cmd = 0.0;
    }
    for (auto &cmd : joint_commands_velocity_)
    {
      cmd = 0.0;
    }
    // for (auto &cmd : joint_commands_position_robotiq_)
    // {
    //   cmd = 0.0;
    // }
    // for (auto &cmd : joint_commands_velocity_robotiq_)
    // {
    //   cmd = 0.0;
    // }

    RCLCPP_INFO(node_->get_logger(), "on_deactivate: hardware disattivato.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    is_active_ = false;
    for (auto &cmd : joint_commands_position_)
    {
      cmd = 0.0;
    }
    for (auto &cmd : joint_commands_velocity_)
    {
      cmd = 0.0;
    }
    // for (auto &cmd : joint_commands_position_robotiq_)
    // {
    //   cmd = 0.0;
    // }
    // for (auto &cmd : joint_commands_velocity_robotiq_)
    // {
    //   cmd = 0.0;
    // }
    RCLCPP_INFO(node_->get_logger(), "on_shutdown: hardware spento.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Called when the hardware interface enters an error state.
   * @param previous_state The lifecycle state prior to the error.
   * @return CallbackReturn::SUCCESS.
  */
  hardware_interface::CallbackReturn CoppeliaSimHardwareInterface::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    is_active_ = false;
    RCLCPP_ERROR(rclcpp::get_logger("CoppeliaSimHardwareInterface"), "Errore in hardware interface.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Esportiamo le interfacce di stato
  std::vector<hardware_interface::StateInterface> CoppeliaSimHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      state_interfaces.emplace_back(joint_names_[i], "position", &joint_positions_[i]);
      state_interfaces.emplace_back(joint_names_[i], "velocity", &joint_velocities_[i]);
      state_interfaces.emplace_back(joint_names_[i], "effort", &joint_efforts_[i]);
    }
    // Stato del gripper
    for (size_t i = 0; i < gripper_joint_names_.size(); ++i)
    {
      state_interfaces.emplace_back(gripper_joint_names_[i], "position", &joint_positions_robotiq_[i]);
      state_interfaces.emplace_back(gripper_joint_names_[i], "velocity", &joint_velocities_robotiq_[i]);
    }

    return state_interfaces;
  }

  /**
   * @brief Exports the command interfaces (position, velocity) for each joint.
   * @return A vector of hardware_interface::CommandInterface objects.
  */
  // Esportiamo le interfacce di comando
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

  // Leggiamo i valori dai messaggi di JointState e li mettiamo nei buffer
  hardware_interface::return_type CoppeliaSimHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Innesca i callback in coda (quindi la jointStateCallback se ci sono msg)
    rclcpp::spin_some(node_);

    if (last_joint_state_ != nullptr)
    {
      // Copiamo i dati dal msg nei nostri buffer
      for (size_t i = 0; i < joint_names_.size(); ++i)
      {
        // Trova l'indice corrispondente nel messaggio, se i nomi corrispondono
        // Oppure presumi che l'ordine coincida
        // Qui facciamo la soluzione "semplice" presupponendo l'ordine uguale
        if (i < last_joint_state_->position.size())
          joint_positions_[i] = last_joint_state_->position[i];
        if (i < last_joint_state_->velocity.size())
          joint_velocities_[i] = last_joint_state_->velocity[i];
        if (i < last_joint_state_->effort.size())
          joint_efforts_[i] = last_joint_state_->effort[i];
      }
    }
    // Legge i dati per il gripper
    if (last_joint_state_robotiq_ != nullptr)
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

  /**
   * @brief Sends the latest joint commands to CoppeliaSim using ROS publishers.
   * @param time The current ROS time.
   * @param period The control loop period.
   * @return return_type::OK if the command was published successfully, return_type::ERROR otherwise.
  */
  hardware_interface::return_type CoppeliaSimHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    std_msgs::msg::Float64MultiArray command_msg;
    command_msg.data = joint_commands_position_;

    if (publisher_)
    {
      publisher_->publish(command_msg);
    }
    else
    {
      return hardware_interface::return_type::ERROR;
    }

    // Comandi per il gripper

    joint_commands_position_robotiq_[1] = -joint_commands_position_robotiq_[0]; // right_knuckle
    joint_commands_position_robotiq_[2] =  joint_commands_position_robotiq_[0]; // left_inner_knuckle
    joint_commands_position_robotiq_[3] = -joint_commands_position_robotiq_[0]; // right_inner_knuckle
    joint_commands_position_robotiq_[4] = -joint_commands_position_robotiq_[0]; // left_finger_tip
    joint_commands_position_robotiq_[5] =  joint_commands_position_robotiq_[0]; // right_finger_tip

    std_msgs::msg::Float64MultiArray command_msg_robotiq;
    command_msg_robotiq.data = joint_commands_position_robotiq_;

    if (publisher_robotiq_)
    {
      //RCLCPP_INFO(node_->get_logger(), "Invio comando gripper: %f", joint_commands_position_robotiq_[0]);
      publisher_robotiq_->publish(command_msg_robotiq);
    }
    else
    {
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

} // namespace coppelia_description

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cr_hw_configuration::CoppeliaSimHardwareInterface, hardware_interface::SystemInterface)
