/// \file gd_manager.hpp GenericDivider filters manager.

#ifndef GD_MANAGER_HPP
#define GD_MANAGER_HPP

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <map>
#include <string>

namespace topic_filters_manager
{
    /// \brief A GenericDivider-specific topic filters manager.
    ///
    /// Does not accept other types of managers.
    /// Works in topic publication mode (no service calls).
    /// Note that all rate publishers are latched.
    ///
    /// The given filter nodes namespace will be prefixed with a '/', i.e.
    /// the manager will publish on '/[filter_name]/divider_rate'.
    ///
    class GenericDividerManager
    {
    public:
        /// \brief Constructor.
        /// 
        /// Creates node handles on demand.
        /// For parameters, assumes the 'topic_filters' is at the same level,
        /// and usually is in /hbba.
        /// For filter handlers, the registration interface assumes the full
        /// name has been given anyway.
        GenericDividerManager();

        /// \brief Sets a filter's rate by name.
        ///
        /// Will try to register the filter if it isn't known yet.
        /// Warns (through ROS_WARN) if the filter doesn't seem to exist (no
        /// subscribers).
        ///
        /// \param name The name of the filter.
        /// \param rate The rate to apply.
        void setRate(const std::string& name, int rate);

    private:
        void parseFiltersParam();

        class FilterHandler
        {
        private:
            ros::Publisher pub_;
            bool first_call_;
        public:
            FilterHandler(const std::string& name);
            void setRate(int rate);
        };

        typedef boost::shared_ptr<FilterHandler>        FilterHandlerPtr;
        typedef std::map<std::string, FilterHandlerPtr> HandlersMap;

        HandlersMap map_;

    };

};

#endif

