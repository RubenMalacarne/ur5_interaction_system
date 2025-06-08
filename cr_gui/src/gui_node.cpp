#include "cr_gui/gui_node.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDateTime>
#include <QMetaObject>
#include <QScrollBar>

namespace cr::gui
{

    GuiNode::GuiNode()
        : QMainWindow(nullptr), Node("cr_gui_viewer")
    {
        /* ---------- IMAGE -------------------------------------------------- */
        image_widget_ = new ImageWidget(this);

        /* ---------- STATUS BLOCK (right) ----------------------------------- */
        title_label_ = new QLabel("STARTING APPLICATION...", this);
        title_label_->setStyleSheet("color:white; font: 700 16px 'Consolas';");
        title_label_->setAlignment(Qt::AlignCenter);

        target_label_ = new QLabel(this);
        target_label_->setStyleSheet("color:white; font: 14px 'Consolas';");
        target_label_->setAlignment(Qt::AlignCenter);
        target_label_->hide();

        auto *status_v = new QVBoxLayout();
        status_v->addStretch(); // ↑ spazio flessibile in alto
        status_v->addWidget(title_label_);
        status_v->addSpacing(12);
        status_v->addWidget(target_label_);
        status_v->addStretch(); // ↓ spazio flessibile in basso

        // auto *top_h = new QHBoxLayout();
        // top_h->addWidget(image_widget_, 2);
        // top_h->addLayout(status_v, 1);

        /* ---------- BAR: phase + percentage + progress --------------------- */
        phase_label_ = new QLabel("Waiting…", this);
        phase_label_->setStyleSheet("color:white; font: 12px 'Arial';");

        percent_label_ = new QLabel("0%", this);
        percent_label_->setStyleSheet("color:white; font: 12px 'Arial';");

        progress_bar_ = new QProgressBar(this);
        progress_bar_->setRange(0, 100);
        progress_bar_->setValue(0);
        progress_bar_->setTextVisible(false);
        progress_bar_->setFixedHeight(10);
        progress_bar_->setStyleSheet(R"(
        QProgressBar        { background:#000; border:1px solid #202020; }
        QProgressBar::chunk { background:#ffffff; }
    )");

        auto *bar_row = new QHBoxLayout();
        bar_row->addWidget(percent_label_);

        auto *bar_wrapper = new QVBoxLayout();
        bar_wrapper->addLayout(bar_row);
        bar_wrapper->addWidget(progress_bar_);
        bar_container_ = new QWidget(this); // Aggiungi questo membro alla tua classe
        bar_container_->setLayout(bar_wrapper);
        bar_container_->setVisible(false); // Nascondi di default

        /* ---------- LOG ---------------------------------------------------- */
        log_widget_ = new QTextEdit(this);
        log_widget_->setReadOnly(true);
        log_widget_->setStyleSheet(
            "background:#1e1e1e; color:#d4d4d4; font:11px 'Courier';");

        /* ---------- MAIN LAYOUT ------------------------------------------- */
        auto *main_widget = new QWidget(this);
        auto *main_v = new QVBoxLayout(main_widget);
        main_v->addWidget(image_widget_, 2);
        main_v->addSpacing(15);
        main_v->addLayout(status_v);
        main_v->addSpacing(6);
        main_v->addWidget(bar_container_);
        main_v->addSpacing(10);
        auto *logs_label = new QLabel("LOGS", this);
        logs_label->setStyleSheet("color: white; font: 700 12px 'Consolas';"); // opzionale: grassetto e font tecnico
        main_v->addWidget(logs_label);
        main_v->addWidget(log_widget_, 2);
        main_v->setContentsMargins(8, 8, 8, 8);

        setCentralWidget(main_widget);
        setStyleSheet("background:#505050;");
        resize(500, 1000);

        /* ---------- ROS subscriptions ------------------------------------- */
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "cr_vision/detected_objects_image", 10,
            std::bind(&GuiNode::imageCb, this, std::placeholders::_1));

        auto qos = rclcpp::QoS(10).transient_local();
        log_sub_ = create_subscription<cr_interfaces::msg::Log>(
            "cr/gui_log", qos,
            std::bind(&GuiNode::statusCb, this, std::placeholders::_1));
    }

    /* ---------------------------------------------------------------------- */
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

    /* ---------------------------------------------------------------------- */
    static QString severityColour(uint8_t severity)
    {
        switch (severity)
        {
        case 1:
            return "#d7ba32"; // WARN
        case 2:
            return "#ff5454"; // ERROR
        default:
            return "#ffffff"; // INFO
        }
    }

    void GuiNode::statusCb(const cr_interfaces::msg::Log::ConstSharedPtr msg)
    {
        /* ---- update title ------------------------------------------------- */
        QString title = QString::fromStdString(msg->main_msg);
        if (title.isEmpty())
            title = "WORKFLOW EXECUTION";
        QMetaObject::invokeMethod(
            title_label_, "setText",
            Qt::QueuedConnection,
            Q_ARG(QString, title.toUpper()));

        /* ---- update phase & bar ------------------------------------------- */
        if (msg->percentage > 0)
        {
            QMetaObject::invokeMethod(
                bar_container_, "setVisible",
                Qt::QueuedConnection,
                Q_ARG(bool, true));

            QMetaObject::invokeMethod(
                progress_bar_, "setValue",
                Qt::QueuedConnection,
                Q_ARG(int, static_cast<int>(msg->percentage)));

            QString pct_txt = QString("%1%").arg(msg->percentage);
            QMetaObject::invokeMethod(
                percent_label_, "setText",
                Qt::QueuedConnection,
                Q_ARG(QString, pct_txt));
        }
        else
        {
            QMetaObject::invokeMethod(
                bar_container_, "setVisible",
                Qt::QueuedConnection,
                Q_ARG(bool, false));
        }

        /* ---- update target id (optional) --------------------------------- */
        if (msg->target_id >= 0)
        {
            QString t = QString("target cube: %1").arg(msg->target_id);
            QMetaObject::invokeMethod(
                target_label_, "setText",
                Qt::QueuedConnection,
                Q_ARG(QString, t));
            if (!target_label_->isVisible())
                target_label_->setVisible(true);
        }
        else if (target_label_->isVisible())
        {
            target_label_->setVisible(false);
        }

        /* ---- append coloured log line ------------------------------------ */
        QString log_msg = QString::fromStdString(msg->log_msg);
        if (!log_msg.isEmpty())
        {
            QString time = QDateTime::currentDateTime().toString("[hh:mm:ss] ");
            QString colour = severityColour(msg->severity);
            QString html = QString("<span style='color:%1;'>%2%3</span><br>")
                               .arg(colour, time, log_msg);

            QMetaObject::invokeMethod(
                log_widget_, "insertHtml",
                Qt::QueuedConnection,
                Q_ARG(QString, html));
        }

        /* ---- auto-scroll (lambda first, then connection) ----------------- */
        QMetaObject::invokeMethod(
            log_widget_,
            [lw = log_widget_]()
            {
                if (auto *bar = lw->verticalScrollBar())
                    bar->setValue(bar->maximum());
            },
            Qt::QueuedConnection);
    }

} // namespace cr::gui
