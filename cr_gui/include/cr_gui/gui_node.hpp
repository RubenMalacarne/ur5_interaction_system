#ifndef CR_GUI___GUI_NODE_HPP_
#define CR_GUI___GUI_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QMainWindow>
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

        ImageWidget *image_widget_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    };
} // namespace cr::gui

#endif