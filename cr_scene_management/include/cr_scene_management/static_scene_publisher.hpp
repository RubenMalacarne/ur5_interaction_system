#ifndef CR_SCENE_MANAGEMENT_STATIC_SCENE_PUBLISHER_HPP
#define CR_SCENE_MANAGEMENT_STATIC_SCENE_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace cr {
namespace scene_management {

    class StaticScenePublisher : public rclcpp::Node
    {
    public:
        explicit StaticScenePublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~StaticScenePublisher() = default;

    private:
        void publishStaticScene();

        // Timer per pubblicare la scena statica
        rclcpp::TimerBase::SharedPtr timer_;
        // Publisher che manderà in broadcast i diff di planning scene (se vuoi farlo manualmente)
        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
    };

}  // namespace scene_management
}  // namespace cr

#endif // CR_SCENE_MANAGEMENT_STATIC_SCENE_PUBLISHER_HPP