#include <ros/ros.h>
#include <QtGui/QMainWindow>
#include <QApplication>
#include <QTableWidget>
#include <QString>
#include "hbba_msgs/Desire.h"
#include "hbba_msgs/AddDesires.h"
#include "hbba_msgs/ResourcesSet.h"
#include "hbba_msgs/SetResourceMax.h"

class Ui_MainWindow;

class MainWindow : public QMainWindow
{
Q_OBJECT;

public:
	MainWindow();
	~MainWindow();

signals:
	void resourceMaxReceived(QString, int);

private slots:
	void sendDesireToCore();
	void displayDesire(hbba_msgs::Desire & msg);
	void displayResourceMax(QString res_id, int res_value);
	void setResourceMax();

private:
	void resourceMaxReceivedCallBack(const hbba_msgs::ResourcesSetConstPtr & res);
	
private:
	ros::NodeHandle nh_;
	ros::Subscriber resourceMax_;
	Ui_MainWindow * ui_;
};
