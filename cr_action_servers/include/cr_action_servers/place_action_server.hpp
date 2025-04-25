#ifndef PLACE_ACTION_SERVER_HPP_
#define PLACE_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cr_interfaces/action/place.hpp>
#include <cr_interfaces/srv/allow_collision.hpp>
#include <cr_interfaces/srv/attach_object.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace cr {
namespace action_servers {

    class PlaceActionServer : public rclcpp::Node
    {
    public:

        explicit PlaceActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:

        using Place = cr_interfaces::action::Place;
        using GoalHandlePlace = rclcpp_action::ServerGoalHandle<Place>;

        rclcpp_action::Server<cr_interfaces::action::Place>::SharedPtr action_server_;
        rclcpp::Client<cr_interfaces::srv::AllowCollision>::SharedPtr allow_collision_client_;
        rclcpp::Client<cr_interfaces::srv::AttachObject>::SharedPtr attach_object_client_;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

        // Action Callbacks
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Place::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePlace> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandlePlace> goal_handle);

        // Main execution logic
        void execute(const std::shared_ptr<GoalHandlePlace> goal_handle);

        // Helper methdos
        bool lift_from_table();
        bool move_to_pre_place_pose(const geometry_msgs::msg::Point point);
        bool approach_place_pose(const cr_interfaces::msg::ObjectInfo object_info, const geometry_msgs::msg::Point point);
        bool open_gripper(const cr_interfaces::msg::ObjectInfo object_info);
        bool retreat_from_place();

        double approach_distance_;
        double pre_place_height_; 
    }; 

} // namespace action_servers
} //namespace cr

#endif 