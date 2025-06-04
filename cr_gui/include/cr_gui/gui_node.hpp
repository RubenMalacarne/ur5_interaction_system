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

class GuiNode : public QMainWindow, public rclcpp::Node
{
    Q_OBJECT
public:
    GuiNode();

private:
    /* ==== ROS callbacks ================================================= */
    void imageCb  (const sensor_msgs::msg::Image::ConstSharedPtr  msg);
    void statusCb (const cr_interfaces::msg::Log::ConstSharedPtr msg);

    /* ==== widgets ======================================================= */
    ImageWidget*  image_widget_;   // left live image
    QLabel*       title_label_;    // messaggio principale
    QLabel*       target_label_;   // "target cube: X" (hidden if X<0)
    QLabel*       phase_label_;    // current phase
    QProgressBar* progress_bar_;   // 0-100 %
    QLabel* percent_label_;
    QTextEdit*    log_widget_;     // coloured log history
    QWidget* bar_container_;
    

    /* ==== ROS subs ====================================================== */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<cr_interfaces::msg::Log>::SharedPtr  log_sub_;
};

} // namespace cr::gui
#endif
