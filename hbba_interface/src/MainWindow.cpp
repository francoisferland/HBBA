
#include "ui_MainWindow.h"
#include "MainWindow.h"

MainWindow::MainWindow()
{
	ui_ = new Ui_MainWindow();
	ui_->setupUi(this);

	// Subscribe to ROS topic
	resourceMax_ = nh_.subscribe("resource_max", 1, &MainWindow::resourceMaxReceivedCallBack, this);
	
	// Connect Qt Signals/Slots
	connect(ui_->pushButton_sendDesire, SIGNAL(clicked()), this, SLOT(sendDesireToCore()));
	connect(this, SIGNAL(resourceMaxReceived(QString, int)), this, SLOT(displayResourceMax(QString, int)));
	connect(ui_->pushButton_setMax, SIGNAL(clicked()), this, SLOT(setResourceMax()));
}

MainWindow::~MainWindow()
{
	delete ui_;
}

//Qt slots
void MainWindow::displayDesire(hbba_msgs::Desire & msg)
{
	QString display = "Created desire : "; 
	display += QString::fromStdString(msg.id);
	display += "  |  ";
	display += QString::fromStdString(msg.type); 
	display += "  |  ";
	display += QString::number(msg.utility);
	display += "  |  ";
	display += QString::fromStdString(msg.params);
	
	ui_->CreatedDesire->setText(display);
}

void MainWindow::sendDesireToCore()
{
	hbba_msgs::AddDesires request = hbba_msgs::AddDesires();
    request.request.desires.push_back(hbba_msgs::Desire());
	hbba_msgs::Desire&  msg = request.request.desires.back();
	msg.id = ui_->nom->text().toStdString();
	msg.type = ui_->type->text().toStdString();
	msg.utility = ui_->utility->value();
	msg.params = ui_->params->text().toStdString();

	ros::service::call("add_desires", request);
	
	displayDesire(msg);
}

void MainWindow::resourceMaxReceivedCallBack(const hbba_msgs::ResourcesSetConstPtr & res)
{
	int i = res->set.size();
	emit resourceMaxReceived(QString::fromStdString(res->set[i-1].id), res->set[i-1].value);
}

void MainWindow::displayResourceMax(QString res_id, int res_value)
{
	bool newRes = true; //check if new resource type
	for(int j=0; j<ui_->resourceType->count(); j++)
	{
		if(ui_->resourceType->itemText(j) == res_id)
			newRes = false; 
	}
	
	if (newRes)
		ui_->resourceType->addItem(res_id);
}

void MainWindow::setResourceMax()
{
	hbba_msgs::SetResourceMax request = hbba_msgs::SetResourceMax();
	request.request.id = ui_->resourceType->currentText().toStdString();
	request.request.value = ui_->setMax->value();
	
	bool ver = ros::service::call("set_resource_max", request);
	
	if (ver)
		ui_->verif->setText(QString::fromStdString("Resource modified")); 
}

