#include "cr_bt_pick_place/set_gripper_node.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>                   // Necessario per RCLCPP_... logging
#include <cr_motion_core/motion_commander.hpp> // Necessario per MotionCommander e MotionStatus

using namespace cr::bt::pick_place;
using BT::NodeStatus;

SetGripperNode::SetGripperNode(const std::string &name,
                               const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
    if (!config.blackboard->get("ros_node", nh_))
    {
        throw BT::RuntimeError("SetGripperNode: missing 'ros_node' on blackboard. Check your setupBT in the orchestrator.");
    }
    // Leggi l'istanza di MotionCommander dalla blackboard
    if (!config.blackboard->get("motion_commander", commander_))
    {
        throw BT::RuntimeError("SetGripperNode: missing 'motion_commander' on blackboard.");
    }

}

// Implementazione del distruttore (può essere default)
SetGripperNode::~SetGripperNode() = default;

// Definizione delle porte di input/output
BT::PortsList SetGripperNode::providedPorts()
{
    return {
        BT::InputPort<double>("target", "apertura gripper [rad]")
    };
}

// Logica eseguita una sola volta quando il nodo diventa RUNNING
NodeStatus SetGripperNode::onStart()
{
    double target; 

    // Lettura del valore dalla porta di input
    if (!getInput("target", target))
    {
        RCLCPP_ERROR(nh_->get_logger(), "SetGripperNode '%s': missing required input [target]", name().c_str());
        return NodeStatus::FAILURE;
    }

    if (!commander_)
    {
        RCLCPP_ERROR(nh_->get_logger(), "SetGripperNode '%s': motion_commander non valido in onStart.", name().c_str());
        return NodeStatus::FAILURE;
    }

    // 3) Avvia il task asincrono tramite MotionCommander
    // Questo metodo pianifica E esegue. Restituisce una future per monitorare lo stato.
    gripper_task_future_ = commander_->async_set_gripper_joint(target);

    // 4) Controlla se l'avvio dell'operazione è riuscito (la future restituita è valida)
    if (!gripper_task_future_.valid())
    {
        RCLCPP_ERROR(nh_->get_logger(), "SetGripperNode '%s': Fallito avvio task async set_gripper_joint. Controllare i log di MotionCommander per dettagli.", name().c_str());
        return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(nh_->get_logger(), "SetGripperNode '%s': Task set_gripper_joint inviato (target %.3f).", name().c_str(), target);
    // L'operazione è stata avviata, il nodo BT è in esecuzione asincrona
    return NodeStatus::RUNNING;
}

// Logica eseguita ripetutamente mentre il nodo è RUNNING
NodeStatus SetGripperNode::onRunning()
{
    cr::motion_core::MotionStatus motion_status = commander_->get_gripper_motion_status();

    switch (motion_status)
    {
    case cr::motion_core::MotionStatus::RUNNING:
        return NodeStatus::RUNNING;

    case cr::motion_core::MotionStatus::SUCCEEDED:
        RCLCPP_INFO(nh_->get_logger(), "SetGripperNode '%s': Task completato con successo.", name().c_str());
        return NodeStatus::SUCCESS;

    case cr::motion_core::MotionStatus::CANCELLED:
        RCLCPP_WARN(nh_->get_logger(), "SetGripperNode '%s': Task annullato esternamente (es. da onHalted).", name().c_str());
        return NodeStatus::FAILURE;

    case cr::motion_core::MotionStatus::FAILED:
    case cr::motion_core::MotionStatus::PENDING:
    default:
        RCLCPP_ERROR(nh_->get_logger(), "SetGripperNode '%s': Task fallito o stato inatteso (%d).", name().c_str(), static_cast<int>(motion_status));
        return NodeStatus::FAILURE;
    }
}

// Logica eseguita quando il nodo viene haltato mentre è RUNNING
void SetGripperNode::onHalted()
{
    RCLCPP_WARN(nh_->get_logger(), "SetGripperNode '%s': Halt richiesto. Richiesta di annullamento task gripper.", name().c_str());
    if (commander_)
    {
        commander_->cancel_gripper_execution();
    }
}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<SetGripperNode>("SetGripper");
}
