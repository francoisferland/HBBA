
#include <QtGui/QApplication>
#include "MainWindow.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gui_node");
	QApplication app(argc, argv);
	MainWindow mainWindow;

	// Connect Qt signals/slots
	app.connect( &app, SIGNAL( lastWindowClosed() ), &app, SLOT( quit() ) );

	mainWindow.show();

	// Here start the ROS events loop
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();

	// Now wait for application to finish
	int r = app.exec();// MUST be called by the Main Thread

	spinner.stop();

	// All done! Closing...
	return r;
}
