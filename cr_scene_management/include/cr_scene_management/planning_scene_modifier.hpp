#ifndef CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP
#define CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP

#include <rclcpp/rclcpp.hpp>
#include <cr_interface/msg/object_info.hpp>
#include <cr_interface/srv/allow_collision.hpp>
#include <cr_interface/srv/attach_object.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

namespace cr {
namespace scene_management {

    class PlanningSceneModifier : public rclcpp::Node
    {
    public:
        explicit PlanningSceneModifier(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        // Sottoscrizione per spawn object
        void spawnObject(const cr_interface::msg::ObjectInfo::SharedPtr object_info);
        // Callback service per allow collision
        void allowCollision(
            const std::shared_ptr<cr_interface::srv::AllowCollision::Request> request,
            std::shared_ptr<cr_interface::srv::AllowCollision::Response> response
        );
        // Callback service per attach object
        void attachObject(
            const std::shared_ptr<cr_interface::srv::AttachObject::Request> request,
            std::shared_ptr<cr_interface::srv::AttachObject::Response> response
        );

        // Publisher, subscriber, service
        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
        rclcpp::Subscription<cr_interface::msg::ObjectInfo>::SharedPtr object_info_sub_;
        rclcpp::Service<cr_interface::srv::AllowCollision>::SharedPtr allow_collision_srv_;
        rclcpp::Service<cr_interface::srv::AttachObject>::SharedPtr attach_object_srv_;
    };

} // namespace scene_management
} // namespace cr

#endif // CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP
