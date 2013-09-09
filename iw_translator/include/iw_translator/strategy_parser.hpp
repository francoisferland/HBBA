#ifndef STRATEGY_PARSER_HPP
#define STRATEGY_PARSER_HPP

#include <hbba_msgs/Strategy.h>
#include <ros/ros.h>

namespace iw_translator
{
    /// \brief A class to parse HBBA Strategies from a YAML description found in
    /// a ROS parameter structure.
    ///
    /// The expected strategy structure mimics the one found in hbba_synth, with
    /// extra fields for the full script source and bringup/bringdown function
    /// names.
    /// Here's a full description of a single strategy object:
    /// 
    /// {
    ///     name:         Unique identifier of the strategy (string)
    ///     class:        Desire class fulfilled by the strategy (string)
    ///     utility:      Utility value (u_ik) (float)
    ///     costs:        A map of cost definitions (c_ij)
    ///     dependencies: A map of requested utility in other classes (r_ik)
    ///     source:       Script source for bringup/bringdown functions (string)
    ///     bringup:      Bringup function called at activation (string)
    ///     bringdown:    Bringdown function called at deactivation (string)
    /// }
    //
    /// Note: only the name, class and utility are mandatory, everything else
    /// can be omitted and will still result in a valid (although unuseful) 
    /// strategy.
    /// Both costs and dependencies follow the same map format where keys are
    /// strings referring to desire classes and values are floats of their
    /// usage, ex. {CPU: 100, Planner: 1}.
    ///
    /// 
    class StrategyParser
    {
    public:
        /// \brief Parse a single strategy from the given ROS parameter.
        ///
        /// The output structure will be left untouched if parsing errors
        /// occured.
        ///
        /// \param node  The XMLRPC value usually obtained from the ROS Parameter
        ///              server.
        ///              Expects a TypeStruct.
        ///              NOTE: Should be const, but member access in the XMLRPC
        ///              library somehow isn't.
        /// \param strat Reference to the output structure.
        /// \return      False if a parsing error occured.
        ///              Will also be signaled with ROS_ERROR.
        ///
        static bool parseSingle(
            XmlRpc::XmlRpcValue& node, 
            hbba_msgs::Strategy& strat);

        /// \brief Parse an array of strategies from the given ROS parameter.
        ///
        /// This method calls parseSingle(...) on each element found in the
        /// given array.
        /// The output structure will be left untouched if parsing errors
        /// occured.
        ///
        /// \param node   The XMLRPC value usually obtained from the ROS
        ///               Parameter server.
        ///               Expects a TypeArray.
        ///               NOTE: Should be const, but member access in the XMLRPC
        ///               library somehow isn't.
        /// \param strats Reference to the output vector.
        /// \return       False if a parsing error occured.
        ///               Will also be signaled with ROS_ERROR.
        ///
        static bool parseArray(
            XmlRpc::XmlRpcValue& node,
            std::vector<hbba_msgs::Strategy>& strats);

        /// \brief Parse a numerical value from a TypeInt or TypeDouble.
        template <class T>
        static bool parseNumber(XmlRpc::XmlRpcValue& node, T& v);

        /// \brief Parse a numerical value from a TypeInt or TypeDouble found in
        /// a struct with the given tag..
        template <class T>
        static bool parseNumber(
            XmlRpc::XmlRpcValue& node, 
            const char* tag, 
            T& v);

        /// \brief Parse a string from a tag-specified sub-element of the given
        /// object.
        ///
        /// \return False if parsing has failed.
        ///
        static bool parseString(
            XmlRpc::XmlRpcValue& node,
            const char* tag,
            std::string& out);

        /// \brief Parse a TypeStruct into a vector of ResourceUsage.
        ///
        /// Can also be used for resource caps definitions.
        ///
        /// Used for costs and dependencies.
        /// Only warn if the given object is not a TypeStruct or a cost value
        /// is not numerical.
        /// Invalid structures will not affect the output vector.
        static void parseCosts(
            XmlRpc::XmlRpcValue& costs,
            std::vector<hbba_msgs::ResourceUsage>& out);

    private:
        // Static string identifiers for YAML structure:
        static const char* TAG_NAME;
        static const char* TAG_CLASS;
        static const char* TAG_UTIL;
        static const char* TAG_COSTS;
        static const char* TAG_DEPS;
        static const char* TAG_SRC;
        static const char* TAG_BUP;
        static const char* TAG_BDN;

    };
}

#endif

