#include "iw_console.hpp"
#include <ros/ros.h>
#include <topic_filters_manager/GetFilters.h>
#include <iw_msgs/DesiresSet.h>
#include <script_engine/EvalScript.h>

using namespace iw_console;

IMPLEMENT_APP_NO_MAIN(iwc_app);

desires_list::desires_list(wxFrame* parent):
	wxListCtrl(parent)
{
	SetWindowStyleFlag(wxLC_REPORT);
	InsertColumn(0, _T("ID"));
	InsertColumn(1, _T("Type"));
	InsertColumn(2, _T("Utility"));
	InsertColumn(3, _T("Params"));

	ros::NodeHandle n;
	sub_ = n.subscribe("desires_set", 10, &desires_list::sub_cb, this);
}

void desires_list::sub_cb(const iw_msgs::DesiresSet::ConstPtr& msg)
{
	DeleteAllItems();
	typedef std::vector<iw_msgs::Desire> v_t;
	v_t::const_iterator i = msg->desires.begin();
	int row = 0;
	for (; i != msg->desires.end(); ++i)
	{
		wxListItem item;
		item.SetId(row);
		InsertItem(item);
		SetItem(row, 0, wxString(i->id.c_str(), wxConvUTF8));
		SetItem(row, 1, wxString(i->type.c_str(), wxConvUTF8));
		wxString u;
		u << i->utility;
		SetItem(row, 2, u);
		SetItem(row, 3, wxString(i->params.c_str(), wxConvUTF8));
		row++;
	}

}

filters_list::filters_list(wxFrame* parent):
	wxListCtrl(parent),
	timer_(new wxTimer(this))
{
	SetWindowStyleFlag(wxLC_REPORT);
	InsertColumn(0, _T("Filter"));
	InsertColumn(1, _T("State"));

	ros::NodeHandle n;
	srv_ = n.serviceClient<topic_filters_manager::GetFilters>("get_filters");

	timer_->Start(1000);
	Connect(timer_->GetId(), wxEVT_TIMER,
		wxTimerEventHandler(filters_list::timer_cb), NULL, this);

}

void filters_list::timer_cb(wxTimerEvent& evt)
{
	topic_filters_manager::GetFilters req;
	if (ros::service::call("get_filters", req))
	{
		DeleteAllItems();
		typedef std::vector<std::string> v_t;
		const v_t& filters = req.response.filters;
		const v_t& states = req.response.states;
		v_t::const_iterator i = filters.begin();
		v_t::const_iterator j = states.begin();
		int row = 0;
		for (; i != filters.end(); ++i, ++j)
		{
			wxListItem item;
			item.SetId(row);
			InsertItem(item);
			SetItem(row, 0, wxString(i->c_str(), wxConvUTF8));
			SetItem(row, 1, wxString(j->c_str(), wxConvUTF8));
			row++;
		}
	}
	else
		ROS_ERROR("Can't call get_filters service.");
}

cmd_prompt::cmd_prompt(wxFrame* parent):
	wxTextCtrl(parent, -1, _T(""), wxPoint(), wxSize(160, 32),
		wxTE_PROCESS_ENTER)
{
	wxTextAttr attr(
		wxColour(), 
		wxNullColour, 
		wxFont(
			11, 
			wxFONTFAMILY_ROMAN,
			wxFONTSTYLE_NORMAL,
			wxFONTWEIGHT_NORMAL));
	SetDefaultStyle(attr);

	Connect(GetId(), wxEVT_COMMAND_TEXT_ENTER,
		wxCommandEventHandler(cmd_prompt::on_text_enter), NULL, this);

}

void cmd_prompt::on_text_enter(wxCommandEvent&)
{
	std::string cmd(GetValue().mb_str());
	SetValue(_T(""));

	script_engine::EvalScript req;
	req.request.source = cmd;
	ros::service::call("eval_script", req);
}

main_frame::main_frame(wxFrame* parent = (wxFrame*) NULL):
	wxFrame(parent, -1, _T("iw_console")),
	sizer_(new wxBoxSizer(wxVERTICAL)),
	desires_list_(new desires_list(this)),
	filters_list_(new filters_list(this)),
	cmd_prompt_(new cmd_prompt(this)),
	timer_(new wxTimer(this))
{
	sizer_->Add(desires_list_, 1, wxEXPAND | wxALL, 1);
	sizer_->Add(filters_list_, 1, wxEXPAND | wxALL, 1);
	sizer_->Add(cmd_prompt_, 0, wxEXPAND);

	SetSizer(sizer_);

	timer_->Start(500);
	Connect(timer_->GetId(), wxEVT_TIMER,
		wxTimerEventHandler(main_frame::timer_cb), NULL, this);

}

void main_frame::timer_cb(wxTimerEvent&)
{
	ros::spinOnce();
}

bool iwc_app::OnInit()
{
	main_frame* frame = new main_frame();
	frame->Show(true);
	SetTopWindow(frame);

	return true;

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "iw_console");
	
	return wxEntry(argc, argv);

}

