#ifndef CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP
#define CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP

#include <rclcpp/rclcpp.hpp>
#include <cr_interfaces/msg/object_info_array.hpp>
#include <cr_interfaces/srv/allow_collision.hpp>
#include <cr_interfaces/srv/attach_object.hpp>
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
        void spawnObjects(const cr_interfaces::msg::ObjectInfoArray::SharedPtr detected_objects);
        // Callback service per allow collision
        void allowCollision(
            const std::shared_ptr<cr_interfaces::srv::AllowCollision::Request> request,
            std::shared_ptr<cr_interfaces::srv::AllowCollision::Response> response
        );
        // Callback service per attach object
        void attachObject(
            const std::shared_ptr<cr_interfaces::srv::AttachObject::Request> request,
            std::shared_ptr<cr_interfaces::srv::AttachObject::Response> response
        );

        // Publisher, subscriber, service
        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
        rclcpp::Subscription<cr_interfaces::msg::ObjectInfoArray>::SharedPtr obj_detection_result_sub_;
        rclcpp::Service<cr_interfaces::srv::AllowCollision>::SharedPtr allow_collision_srv_;
        rclcpp::Service<cr_interfaces::srv::AttachObject>::SharedPtr attach_object_srv_;
    };

} // namespace scene_management
} // namespace cr

#endif // CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP
