#include "cr_vision/human_proximity_monitor.hpp"

namespace cr::vision
{

    HumanProximityMonitor::HumanProximityMonitor(const rclcpp::NodeOptions &options)
        : Node("human_proximity_monitor", options),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // Declare and read the safe distance parameter
        this->declare_parameter<double>("safe_distance", 0.5);
        this->get_parameter("safe_distance", safe_distance_);

        // Publisher: true if a human is within the safe distance
        human_near_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/cr/human_near", rclcpp::QoS(10).reliable());

        // Subscriber to the list of human poses from simulation
        human_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/coppelia/human_poses",
            rclcpp::QoS(10).reliable(),
            std::bind(&HumanProximityMonitor::human_poses_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "HumanProximityMonitor started: safe_distance = %.2f m",
                    safe_distance_);
    }

    void HumanProximityMonitor::human_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        bool human_near = false;

        // Get transform from world (msg frame) to "tool0"
        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_.lookupTransform(
                msg->header.frame_id, // source frame
                "tool0",              // target frame
                tf2::TimePointZero);  // latest available transform
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        // Get end-effector position in XY plane
        double ee_x = tf.transform.translation.x;
        double ee_y = tf.transform.translation.y;

        // Check if any human is within the safe distance (XY only)
        for (const auto &pose : msg->poses)
        {
            double dx = pose.position.x - ee_x;
            double dy = pose.position.y - ee_y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < safe_distance_)
            {
                human_near = true;
                break;
            }
        }

        if (human_near)
        {
            RCLCPP_INFO(this->get_logger(), "Human nearby!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No humans nearby.");
        }

        std_msgs::msg::Bool out_msg;
        out_msg.data = human_near;
        human_near_pub_->publish(out_msg);
    }
} // namespace cr::vision
