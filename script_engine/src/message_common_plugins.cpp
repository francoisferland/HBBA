#include <script_engine_plugins/publisher_topic_arg_base.hpp>
#include <hbba_msgs/EvalScript.h>
#include <hbba_msgs/Boolean.h>
#include <hbba_msgs/UpdateRate.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Duration.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <stdlib.h>
#include <string>

/// \brief Common functions plugin for the script engine.
namespace common_plugins 
{
    /// \brief A basic function to publish on std_msgs/Empty topics.
    namespace pub_empty
    {
        extern void afun(const v8::Arguments&, std_msgs::Empty&)
        {
        }

        const extern char pub_name[] = "pubEmpty";

        typedef script_engine_plugins::publisher_topic_arg_base<
            std_msgs::Empty,
            pub_name,
            afun> PubEmptyPlugin;
    }
    
    /// \brief A basic function to publish booleans on topics.
    namespace pub_boolean
    {
        extern void afun(const v8::Arguments& args, 
            std_msgs::Bool& msg)
        {
            msg.data = args[1]->BooleanValue();
        }

        const extern char pub_name[] = "pubBoolean";

        typedef script_engine_plugins::publisher_topic_arg_base<
            std_msgs::Bool,
            pub_name,	
            afun> PubBooleanPlugin;
    }

    /// \brief A basic function to publish integers on topics.
    namespace pub_int
    {
        template <class IntType, class RosMessage>
        extern void afun(const v8::Arguments& args, 
            RosMessage& msg)
        {
            msg.data = static_cast<IntType>(args[1]->IntegerValue());
        }

        const extern char pub_name_int8[] = "pubInt8";
        const extern char pub_name_int16[] = "pubInt16";
        const extern char pub_name_int32[] = "pubInt32";
        const extern char pub_name_int64[] = "pubInt64";

        typedef script_engine_plugins::publisher_topic_arg_base<
            std_msgs::Int8,
            pub_name_int8,	
            afun<int8_t, std_msgs::Int8> > PubInt8Plugin;
        typedef script_engine_plugins::publisher_topic_arg_base<
            std_msgs::Int16,
            pub_name_int16,	
            afun<int16_t, std_msgs::Int16> > PubInt16Plugin;
        typedef script_engine_plugins::publisher_topic_arg_base<
            std_msgs::Int32,
            pub_name_int32,	
            afun<int32_t, std_msgs::Int32> > PubInt32Plugin;
        typedef script_engine_plugins::publisher_topic_arg_base<
            std_msgs::Int64,
            pub_name_int64,	
            afun<int64_t, std_msgs::Int64> > PubInt64Plugin;
    }

    /// \brief A basic function to publish floats on topics.
    namespace pub_float
    {
        template <class FloatType, class RosMessage>
        extern void afun(const v8::Arguments& args, 
            RosMessage& msg)
        {
            msg.data = static_cast<FloatType>(args[1]->NumberValue());
        }

        const extern char pub_name_float32[] = "pubFloat32";
        const extern char pub_name_float64[] = "pubFloat64";

        typedef script_engine_plugins::publisher_topic_arg_base<
            std_msgs::Float32,
            pub_name_float32,	
            afun<float, std_msgs::Float32> > PubFloat32Plugin;
        typedef script_engine_plugins::publisher_topic_arg_base<
            std_msgs::Float64,
            pub_name_float64,	
            afun<double, std_msgs::Float64> > PubFloat64Plugin;
    }

    /// \brief A basic function to publish strings on topics.
    namespace pub_string
    {
        extern void afun(const v8::Arguments& args, 
            std_msgs::String& msg)
        {
            v8::String::Utf8Value v8_text(args[1]);
            msg.data = std::string(*v8_text);
        }

        const extern char pub_name[] = "pubString";

        typedef script_engine_plugins::publisher_topic_arg_base<
            std_msgs::String,
            pub_name,	
            afun> PubStringPlugin;
    }

    /// \brief A geometry_msgs/PoseStamped publication tool takes as input a 
    /// planar (x, y, theta) pose usually meant as a navigation goal.
    ///
    /// Params:
    ///  0: The output topic's name, taken care of by the base class.
    ///  1: The frame id (String)
    ///  2: x position
    ///  3: y position
    ///  4: theta angle, (around Z).
    namespace pub_nav_goal
    {
        extern void afun(const v8::Arguments& args,
            geometry_msgs::PoseStamped& msg)
        {
            if (args.Length() < 5)
            {
                ROS_ERROR("Not enough arguments in pubNavGoal call.");
                return;
            }

            v8::String::Utf8Value frame_id(args[1]);
            v8::Number* vx = v8::Number::Cast(*args[2]);
            v8::Number* vy = v8::Number::Cast(*args[3]);
            v8::Number* vt = v8::Number::Cast(*args[4]);

            ROS_DEBUG("Sending navigation goal: %s, %f, %f, %f.",
                msg.header.frame_id.c_str(),
                vx->Value(),
                vy->Value(),
                vt->Value());

            msg.header.frame_id = *frame_id;
            msg.header.stamp = ros::Time::now();
            msg.pose.position.x = vx->Value();
            msg.pose.position.y = vy->Value();
            msg.pose.position.z = 0.0;
            msg.pose.orientation = tf::createQuaternionMsgFromYaw(vt->Value());

        }

        const extern char pub_name[] = "pubNavGoal";

        typedef script_engine_plugins::publisher_topic_arg_base<
            geometry_msgs::PoseStamped,
            pub_name,
            afun> PubNavGoalPlugin;
    }

    /// \brief A basic function to publish durations on topics.
    /// 
    /// overloads:
    ///  pubDuration(topic, sec)
    ///   topic: String
    ///   sec: double
    ///  pubDuration(topic, sec, nsec)
    ///   topic: String
    ///   sec: int
    ///   nsec: int
    namespace pub_duration
    {
        extern void afun(const v8::Arguments& args, 
            std_msgs::Duration& msg)
        {
            if (args.Length() == 2)
            {
                msg.data = ros::Duration(args[1]->NumberValue());
            }
            else if (args.Length() == 3)
            {
                msg.data = ros::Duration(args[1]->Int32Value(), args[2]->Int32Value());
            }
            else
            {
                ROS_ERROR("Wrong number of arguments in pubDuration call.");
            }
        }

        const extern char pub_name[] = "pubDuration";

        typedef script_engine_plugins::publisher_topic_arg_base<
            std_msgs::Duration,
            pub_name,	
            afun> PubDurationPlugin;
    }
}

PLUGINLIB_EXPORT_CLASS(
    common_plugins::pub_empty::PubEmptyPlugin,
    script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
    common_plugins::pub_boolean::PubBooleanPlugin,
    script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
    common_plugins::pub_int::PubInt8Plugin,
    script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
    common_plugins::pub_int::PubInt16Plugin,
    script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
    common_plugins::pub_int::PubInt32Plugin,
    script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
    common_plugins::pub_int::PubInt64Plugin,
    script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
    common_plugins::pub_float::PubFloat32Plugin,
    script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
    common_plugins::pub_float::PubFloat64Plugin,
    script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
	common_plugins::pub_string::PubStringPlugin,
	script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
	common_plugins::pub_nav_goal::PubNavGoalPlugin,
	script_engine_plugins::engine_module);
PLUGINLIB_EXPORT_CLASS(
	common_plugins::pub_duration::PubDurationPlugin,
	script_engine_plugins::engine_module);
