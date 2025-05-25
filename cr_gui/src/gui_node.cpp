#include "cr_gui/gui_node.hpp"
#include <QVBoxLayout>
#include <thread>

namespace cr::gui
{
    GuiNode::GuiNode() : QMainWindow(nullptr), Node("cr_gui_viewer")
    {
        image_widget_ = new ImageWidget(this);
        setCentralWidget(image_widget_);
        setWindowTitle("Visualizzatore YOLO");
        resize(800, 600);

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "cr_vision/yolov8_detection_image",  10,
            std::bind(&GuiNode::imageCb, this, std::placeholders::_1)
        );
    }

    void GuiNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try {
            auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            image_widget_->updateImage(cv_ptr->image);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
        }
    }
} // namespace cr::gui


