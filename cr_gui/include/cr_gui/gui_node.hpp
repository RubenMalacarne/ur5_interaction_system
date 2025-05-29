#ifndef CR_GUI___GUI_NODE_HPP_
#define CR_GUI___GUI_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/string.hpp>
#include <QMainWindow>
#include <QSplitter>
#include <QPlainTextEdit>
#include "image_widget.hpp"

namespace cr::gui
{
    class GuiNode : public QMainWindow, public rclcpp::Node
    {
        Q_OBJECT
    public:
        GuiNode();

    private:
        void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void logCb(const std_msgs::msg::String::ConstSharedPtr msg);

        ImageWidget *image_widget_;
        QPlainTextEdit *log_widget_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr log_sub_;
    };
} // namespace cr::gui

#endif