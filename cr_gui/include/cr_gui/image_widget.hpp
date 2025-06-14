/**
 * @file image_widget.hpp
 * @brief Qt widget for displaying OpenCV images with object detections.
 *
 * This widget provides a thread-safe interface to render OpenCV `cv::Mat` frames
 * inside a Qt GUI, using `QLabel` as base and `QPainter` for custom drawing.
 * It is specifically designed to display the real-world scene captured by the camera
 * together with overlaid detection results.
 */

#ifndef CR_GUI___IMAGE_WIDGET_HPP_
#define CR_GUI___IMAGE_WIDGET_HPP_

#include <QLabel>
#include <QMutex>
#include <opencv2/opencv.hpp>

namespace cr::gui
{
    /**
     * @class ImageWidget
     * @brief QLabel-based widget for rendering camera frames with detected object overlays.
     *
     * This class provides a thread-safe mechanism to display OpenCV images using `QPainter`.
     * It is specifically intended to visualize the data stream coming from the camera (the RGB image
     * of the real-world scene, with overlaid information about detected objects).
     *
     * The widget is designed to assist the human operator by presenting in real time what
     * the perception system detects.
     */
    class ImageWidget : public QLabel
    {
        Q_OBJECT

    public:
        /**
         * @brief Constructor.
         * @param parent Pointer to parent QWidget.
         */
        explicit ImageWidget(QWidget *parent = nullptr);

        /**
         * @brief Updates the current image to be displayed.
         * @param frame The new OpenCV frame to render (expected in BGR format).
         *
         * Internally converts the image to RGB and requests a repaint.
         * Thread-safe.
         */
        void updateImage(const cv::Mat &frame);

    protected:
        /**
         * @brief Paint event override to render the current image.
         * @param event The Qt paint event.
         *
         * Uses QPainter to draw the converted `cv::Mat` on the widget.
         */
        void paintEvent(QPaintEvent *event) override;

    private:
        /// Mutex to protect concurrent access to the frame.
        QMutex mtx_;

        /// Current OpenCV image to be displayed.
        cv::Mat frame_;
    };

} // namespace cr::gui

#endif // CR_GUI___IMAGE_WIDGET_HPP_
