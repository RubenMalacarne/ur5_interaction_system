#include "cr_gui/image_widget.hpp"
#include <QPainter>

namespace cr::gui
{

    ImageWidget::ImageWidget(QWidget *parent) : QLabel(parent)
    {
        setMinimumSize(640, 480);
        setAlignment(Qt::AlignCenter);
    }

    void ImageWidget::updateImage(const cv::Mat &frame)
    {
        QMutexLocker locker(&mtx_);
        cv::cvtColor(frame, frame_, cv::COLOR_BGR2RGB);
        update();
    }

    void ImageWidget::paintEvent(QPaintEvent *event)
    {
        QMutexLocker locker(&mtx_);
        if (frame_.empty())
        {
            QLabel::paintEvent(event);
            return;
        }
        QImage img(frame_.data, frame_.cols, frame_.rows,
                   frame_.step, QImage::Format_RGB888);
        QPainter painter(this);
        painter.drawImage(rect(), img);
    }
}