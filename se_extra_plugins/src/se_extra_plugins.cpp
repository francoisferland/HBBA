#include <script_engine_plugins/publisher_topic_arg_base.hpp>
#include <std_msgs/Float64.h>
#include <pluginlib/class_list_macros.hpp>
#include <v8.h>

namespace se_extra_plugins
{
    extern void pubFloat64Fun(const v8::Arguments& args,
                              std_msgs::Float64&   msg)
    {
        if (args.Length() < 2) {
            ROS_ERROR("Not enough arguments to pubFloat64(topic, value).");
            return;
        }

        v8::Number* v = v8::Number::Cast(*args[1]);

        ROS_DEBUG("Sending float value %f", v->Value());

        msg.data = v->Value();

        return;
    }

    const extern char pub_float64_name[] = "pubFloat64";
    typedef script_engine_plugins::publisher_topic_arg_base<
        std_msgs::Float64,
        pub_float64_name,
        pubFloat64Fun> PubFloat64Plugin;

}

PLUGINLIB_EXPORT_CLASS(se_extra_plugins::PubFloat64Plugin,
                       script_engine_plugins::engine_module);
