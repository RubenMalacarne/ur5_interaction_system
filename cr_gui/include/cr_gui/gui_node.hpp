/**
 * @file gui_node.hpp
 * @brief GUI Node for displaying perception data and workflow status in a Qt window.
 *
 * The GUI is intended to support human supervision of the robotic system, making
 * the internal state of the perception and behavior pipeline visible.
 */

#ifndef CR_GUI___GUI_NODE_HPP_
#define CR_GUI___GUI_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cr_interfaces/msg/log.hpp>
#include <cv_bridge/cv_bridge.h>

#include <QMainWindow>
#include <QLabel>
#include <QProgressBar>
#include <QTextEdit>
#include <QWidget>
#include "image_widget.hpp"

namespace cr::gui
{

    /**
     * @class GuiNode
     * @brief Qt window displaying camera feed and real-time workflow status from ROS 2 topics.
     *
     * This class serves as the main GUI component of the robotic system, providing visual feedback
     * to the operator.
     *
     * It subscribes to:
     * - A sensor_msgs::Image topic for the annotated live camera feed
     * - A cr_interfaces::msg::Log topic for workflow updates and system feedback
     */
    class GuiNode : public QMainWindow, public rclcpp::Node
    {
        Q_OBJECT
    public:
        /**
         * @brief Constructor. Initializes the Qt layout and ROS 2 subscriptions.
         */
        GuiNode();

    private:
        /**
         * @brief Callback for receiving camera images.
         * Converts the ROS image to an OpenCV frame and updates the GUI display.
         * @param msg The incoming sensor_msgs::Image message.
         */
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

        /**
         * @brief Callback for receiving log/status messages.
         * Updates the main title, target ID, progress bar and appends log entries to the GUI.
         * @param msg The incoming cr_interfaces::msg::Log message.
         */
        void logCallback(const cr_interfaces::msg::Log::ConstSharedPtr msg);

        /// Widget displaying the annotated camera image.
        ImageWidget *image_widget_;

        /// Main status label displayed at the top.
        QLabel *title_label_;

        /// Label for the current target object.
        QLabel *target_label_;

        /// Horizontal bar showing execution progress.
        QProgressBar *progress_bar_;

        /// Percentage value shown near the progress bar.
        QLabel *percent_label_;

        /// Log viewer for messages received from the workflow.
        QTextEdit *log_widget_;

        /// Container for progress bar and percentage label.
        QWidget *bar_container_;

        /// Subscriber for the camera image topic.
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

        /// Subscriber for workflow log messages.
        rclcpp::Subscription<cr_interfaces::msg::Log>::SharedPtr log_sub_;
    };

} // namespace cr::gui

#endif // CR_GUI___GUI_NODE_HPP_
