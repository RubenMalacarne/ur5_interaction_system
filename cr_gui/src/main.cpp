#include <QApplication>
#include "cr_gui/gui_node.hpp"

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	QApplication app(argc, argv);

	auto window = std::make_shared<cr::gui::GuiNode>();
	window->show();

	std::thread ros_thread([&]()
						   { rclcpp::spin(window); });

	int ret = app.exec();
	rclcpp::shutdown();
	ros_thread.join();
	return ret;
}