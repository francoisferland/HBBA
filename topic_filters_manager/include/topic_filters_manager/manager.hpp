#ifndef TOPIC_FILTERS_MANAGER_HPP
#define TOPIC_FILTERS_MANAGER_HPP

#include "topic_filters_manager/RegisterFilter.h"
#include "topic_filters_manager/GetFilters.h"
#include <ros/ros.h>
#include <string>
#include <vector>
#include <tr1/unordered_map>

/// A note on template specializations:
///
/// Right now, only one type of filter with a single service interface exists: 
/// switch_filter. All of its specializations are found in manager.cpp. These
/// specializations could be defined in separate .cpp files or even entirely
/// different libraries, as long as the final linkage of the topic manager where
/// to find these libraries. 

namespace topic_filters_manager
{
	class filter_handler;

	/// Replaces the old std::pair-based storage.
	struct filter
	{
		typedef std::vector<ros::ServiceClient> proxies_t;

		filter(): state_("undefined") {}
		filter(const std::string& type, const proxies_t& proxies):
			filter_type_(type),
			proxies_(proxies),
			state_("undefined")
		{}

		/// \brief Returns the filter type key.
		const std::string& filter_type() const { return filter_type_; }

		/// \brief Returns a vector of service proxies.
		proxies_t& proxies() { return proxies_; }

		/// \brief Return the current state.
		///
		/// The format is filter type-specific.
		/// NOTE: This state might not reveal the actual state of the filter if
		/// its services have been call from somewhere else than this manager.
		/// The state might also be undefined until the filter has been called. 
		const std::string& state() const { return state_; }

		/// \brief Update a filter's state depending on the request.
		///
		/// Requires specialization based on T, the service type.
		template <class T> void state(const T&);

	private:
		std::string filter_type_;
		proxies_t proxies_;
		/// \brief Dependent on the actual filter type, filled by the specific
		/// handler.
		std::string state_;
	};

	/// This is used to recall the index of a service proxy in a filter
	/// name/proxies vector pair.
	template<class T> extern size_t proxy_index();

	/// \brief Update a filter's state depending on the request.
	///
	/// Needs specialization.
	template<class T> extern inline void update_state(
		const T&, std::string& state);

	class manager
	{
	public:
		//typedef std::pair< std::string, std::vector<ros::ServiceClient> > 
		typedef filter
			filter_t;
		typedef std::tr1::unordered_map<std::string, filter_t> filter_map_t;
		typedef std::tr1::unordered_map<std::string, filter_handler*> 
			handlers_map_t;

		manager();
		~manager();

		/// \brief Register a filter.
		///
		/// \param ns The full name of the namespace containing your filter,
		/// without a trailing "/"
		/// \param type The filter type name.
		void register_filter(const std::string& ns, const std::string& type);

		/// \brief Call a specific filter depending on the service type T.
		template <class T> 
		void call_filter(const std::string& ns, T& req)
		{
            ROS_DEBUG("Setting filter rate for %s...", ns.c_str());
			filter_map_t::iterator i = filter_map_.find(ns);
			if (i != filter_map_.end())
			{
				filter_t& filter = i->second;
				filter.proxies()[proxy_index<T>()].call(req);
				filter.state(req);
			}
			else
				ROS_ERROR("No filter available in namespace %s", ns.c_str());
            ROS_DEBUG("Setting filter rate for %s done.", ns.c_str());
		}


	private:
        void parse_topic_filters_param();

		bool register_filter_srv(RegisterFilter::Request& req, 
			RegisterFilter::Response& res);
		bool get_filters_srv(GetFilters::Request& req, 
			GetFilters::Response& res);

		filter_map_t filter_map_;
		handlers_map_t handlers_map_;

		ros::NodeHandle n_;
		ros::ServiceServer srv_register_filter_;
		ros::ServiceServer srv_get_filters_;

	};

	struct filter_handler
	{
        virtual ~filter_handler() {}
		virtual bool valid(const std::string& ns) = 0;
		virtual void add_proxies(ros::NodeHandle&, 
			const std::string& ns, manager::filter_map_t&) const = 0;
	};


}

#endif

