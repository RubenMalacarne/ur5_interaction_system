#ifndef HUMAN_PROXIMITY_MONITOR_HPP_
#define HUMAN_PROXIMITY_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace cr {
namespace vision {

    class HumanProximityMonitor : public rclcpp::Node
    {
    public:

        explicit HumanProximityMonitor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:

        void human_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
        
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr human_near_pub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr human_poses_sub_;

        double safe_distance_;
        tf2_ros::Buffer        tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
    };

} // namespace vision
} // namespace cr 

#endif