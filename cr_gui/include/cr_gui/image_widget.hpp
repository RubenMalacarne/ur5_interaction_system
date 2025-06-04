#ifndef CR_GUI___IMAGE_WIDGET_HPP_
#define CR_GUI___IMAGE_WIDGET_HPP_

#include <QLabel>
#include <QMutex>
#include <opencv2/opencv.hpp>

namespace cr::gui
{
    class ImageWidget : public QLabel
    {   
        Q_OBJECT
    public:
        explicit ImageWidget(QWidget *parent = nullptr);
        void updateImage(const cv::Mat &frame);

    protected:
        void paintEvent(QPaintEvent *event) override;

    private:
        mutable QMutex mtx_;
        cv::Mat frame_;
    };
} // namespace cr::gui

#endif