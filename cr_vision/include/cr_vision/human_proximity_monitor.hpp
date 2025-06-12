#ifndef CR_VISION_HUMAN_PROXIMITY_MONITOR_HPP_
#define CR_VISION_HUMAN_PROXIMITY_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace cr::vision
{

    /**
     * @file human_proximity_monitor.hpp
     * @brief Declares HumanProximityMonitor node for ROS2.
     *
     * This node listens to human poses from the simulation in CoppeliaSim,
     * transforms them into the robot's end-effector frame, and checks if any
     * human is closer than a safe distance. It publishes a Bool message
     * on "/cr/human_near" indicating true if a human is too close.
     *
     */

    /**
     * @class HumanProximityMonitor
     * @brief Monitors how close humans are to the robot.
     *
     * The node subscribes to a PoseArray topic that provides positions
     * of humans. It uses TF2 to get the transform from the world frame
     * to the end-effector ("tool0") and calculates the horizontal (XY)
     * distance between each human and the robot tool.
     * If any distance is below the configured safe_distance, it
     * publishes true, otherwise false.
     *
     * @ingroup cr_vision_nodes
     */
    class HumanProximityMonitor : public rclcpp::Node
    {
    public:
        /**
         * @brief Create the HumanProximityMonitor node.
         *
         * Declares and reads the "safe_distance" parameter (double, default 0.5 m),
         * creates the "/cr/human_near" publisher and the
         * "/coppelia/human_poses" subscriber, and sets up the TF2 listener.
         *
         * @param options NodeOptions for ROS2 configuration.
         */
        explicit HumanProximityMonitor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        /**
         * @brief Default destructor.
         *
         */
        ~HumanProximityMonitor() override = default;

    private:
        /**
         * @brief Callback for incoming human PoseArray messages.
         *
         * Transforms each human pose into the "tool0" frame, then computes
         * the XY-plane distance between the robot tool and each human.
         * If any human is closer than safe_distance_, it sets human_near to true.
         * Finally, it publishes the result on "/cr/human_near".
         *
         * @param msg Shared pointer to the PoseArray message with human poses.
         */
        void human_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

        /// Publisher: true if a human is within the safe distance
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr human_near_pub_;

        /// Subscriber to the array of human poses from Coppelia simulation
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr human_poses_sub_;

        /// Minimum safe distance (in meters) between robot and human
        double safe_distance_;

        /// TF2 buffer to store transforms
        tf2_ros::Buffer tf_buffer_;

        /// TF2 listener to fill the buffer
        tf2_ros::TransformListener tf_listener_;
    };

} // namespace cr::vision

#endif // CR_VISION_HUMAN_PROXIMITY_MONITOR_HPP_