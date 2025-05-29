#include "cr_gui/gui_node.hpp"
#include <QVBoxLayout>
#include <thread>

namespace cr::gui
{
    GuiNode::GuiNode() : QMainWindow(nullptr), Node("cr_gui_viewer")
    {
        // Widget per l'immagine
        image_widget_ = new ImageWidget(this);

        // Widget per il log
        log_widget_ = new QPlainTextEdit(this);
        log_widget_->setReadOnly(true);
        log_widget_->setMaximumBlockCount(100);

        // Splitter orizzontale
        QSplitter *split = new QSplitter(Qt::Horizontal, this);
        split->addWidget(image_widget_);
        split->addWidget(log_widget_);
        split->setStretchFactor(0, 1);
        split->setStretchFactor(1, 1);

        setCentralWidget(split);
        setWindowTitle("Visualizzatore YOLO con Log");
        resize(1000, 600);

        // Subscription per l'immagine
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "cr_vision/yolov8_detection_image", 10,
            std::bind(&GuiNode::imageCb, this, std::placeholders::_1));

        // Subscription per i log
        log_sub_ = create_subscription<std_msgs::msg::String>(
            "cr/gui_log", 10,
            std::bind(&GuiNode::logCb, this, std::placeholders::_1));
    }

    void GuiNode::imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        try
        {
            auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            image_widget_->updateImage(cv_ptr->image);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
        }
    }

    void GuiNode::logCb(const std_msgs::msg::String::ConstSharedPtr msg)
    {

        QString text = QString("%1")
                           .arg(QString::fromStdString(msg->data));

        // Appendi in modo thread-safe
        QMetaObject::invokeMethod(
            log_widget_, "appendPlainText",
            Qt::QueuedConnection,
            Q_ARG(QString, text));
    }

} // namespace cr::gui
