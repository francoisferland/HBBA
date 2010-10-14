#ifndef IW_CONSOLE_HPP
#define IW_CONSOLE_HPP

#include <wx/wx.h>
#include <wx/listctrl.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
#include <wx/sizer.h>
#include <iw_msgs/DesiresSet.h>
#include <ros/ros.h>

namespace iw_console
{
	class desires_list: public wxListCtrl
	{
	public:
		desires_list(wxFrame* parent);

		void sub_cb(const iw_msgs::DesiresSet::ConstPtr& msg);
	
	private:
		ros::Subscriber sub_;

	};

	class filters_list: public wxListCtrl
	{
	public:
		filters_list(wxFrame* parent);

		void timer_cb(wxTimerEvent& evt);

	private:
		wxTimer* timer_;
		ros::ServiceClient srv_;

	};

	class cmd_prompt: public wxTextCtrl
	{
	public:
		cmd_prompt(wxFrame* parent);

	private:
		void on_text_enter(wxCommandEvent&);

	};

	class main_frame: public wxFrame
	{
	public:
		main_frame(wxFrame*);

		void timer_cb(wxTimerEvent&);

	private:
		wxBoxSizer* sizer_;
		desires_list* desires_list_;
		filters_list* filters_list_;
		cmd_prompt* cmd_prompt_;
		wxTimer* timer_;

	};

	class iwc_app: public wxApp
	{
	public:
		virtual bool OnInit();
	};

	DECLARE_APP(iwc_app);

};

#endif

