#ifndef PICK_ACTION_SERVER_HPP_
#define PICK_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cr_interfaces/action/pick.hpp>
#include <cr_interfaces/msg/object_info.hpp>
#include <cr_interfaces/srv/allow_collision.hpp>
#include <cr_interfaces/srv/attach_object.hpp>

namespace cr {
namespace action_servers {

    class PickActionServer : public rclcpp::Node
    {
    public:

        explicit PickActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    private:

        using Pick = cr_interfaces::action::Pick;
        using GoalHandlePick = rclcpp_action::ServerGoalHandle<Pick>;

        rclcpp_action::Server<Pick>::SharedPtr action_server_;
        rclcpp::Client<cr_interfaces::srv::AllowCollision>::SharedPtr allow_collision_client_;
        rclcpp::Client<cr_interfaces::srv::AttachObject>::SharedPtr attach_object_client_;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
        
        // Action Callbacks
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Pick::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePick> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandlePick> goal_handle);

        // Main execution logic
        void execute(const std::shared_ptr<GoalHandlePick> goal_handle);

        // Helper methdos
        bool reach_pre_grasp_height(const cr_interfaces::msg::ObjectInfo &object_info);
        bool reach_pre_grasp(const cr_interfaces::msg::ObjectInfo &object_info);
        bool move_above_object(const cr_interfaces::msg::ObjectInfo &object_info);
        bool approach_object(const cr_interfaces::msg::ObjectInfo &object_info);
        bool pick_object(const cr_interfaces::msg::ObjectInfo &object_info);
        bool retreat();
        
        double pre_approach_distance_;
        double approach_distance_;
        double retreat_offset_;
    };
    
} // namespace action_servers
} // namespace cr

#endif