#include "cr_vision/human_proximity_monitor.hpp"

namespace cr {
namespace vision {

/**
 * @brief Implementation for the HumanProximityMonitor node.
 *
 * Initializes the node, declares and retrieves the safe distance parameter, and sets up
 * the TF2 buffer and listener, along with publishers and subscribers.
 * 
 */
    HumanProximityMonitor::HumanProximityMonitor(const rclcpp::NodeOptions &options)
        : Node("human_proximity_monitor", options),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)    
    {
        // Parameter for the safe distance
        this->declare_parameter<double>("safe_distance", 0.5);
        this->get_parameter("safe_distance", safe_distance_);
    
        // Publisher: true = umano vicino
        human_near_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/cr/human_near", rclcpp::QoS(10).reliable());
    
        // Subscriber all'array di pose umane
        human_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/coppelia/human_poses",
            rclcpp::QoS(10).reliable(),
            std::bind(&HumanProximityMonitor::human_poses_callback, this, std::placeholders::_1));
    
        RCLCPP_INFO(this->get_logger(),
                    "HumanProximityMonitor avviato: safe_distance=%.2f m",
                    safe_distance_);
    }
    
    /**
     * @brief Callback function that processes an array of human poses.
     *
     * Computes the distance between the robot's end-effector (`tool0`) and each human pose
     * to determine if a human is within the configured safe distance. Publishes the result
     * as a boolean on the `/cr/human_near` topic.
     *
     * @param msg Shared pointer to the received PoseArray message.
     */
    void HumanProximityMonitor::human_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        bool human_near = false;
    
        // Trasformazione world -> tool0
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform(
                msg->header.frame_id,     // sistema di riferimento in cui si intende esprimere il punto
                "tool0",                  // frame del quale si cerca l'orientazione
                tf2::TimePointZero);      // instante temporale di cui si intende avere la trasformazione
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }
    
        // Estrazione posizione di tool0
        double ee_x = tf.transform.translation.x;
        double ee_y = tf.transform.translation.y;

        // Iterazione sulle pose umane e calcolo distanza sul piano XY
        for (const auto &pose : msg->poses) {
            double dx = pose.position.x - ee_x;
            double dy = pose.position.y - ee_y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist < safe_distance_) {
                human_near = true;
                //RCLCPP_INFO(this->get_logger(), "Rilevato uomo nelle vicinanze! Distanza = %.2f", dist);
                // RCLCPP_INFO(this->get_logger(), "tool0 = [%.2f %.2f]", ee_x , ee_y);
                // RCLCPP_INFO(this->get_logger(), "human = [%.2f %.2f]", pose.position.x , pose.position.y);
                // RCLCPP_INFO(this->get_logger(), "ditance = [%.2f]", dist);
                break;
            } 
        }

        // Per debug
        if(human_near){
            RCLCPP_INFO(this->get_logger(), "Rilevato uomo nelle vicinanze!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Nessuno uomo nelle vicinanze."); 
        }

        std_msgs::msg::Bool out_msg;
        out_msg.data = human_near;
        human_near_pub_->publish(out_msg);
    }

} // namespace vision
} // namespace cr 