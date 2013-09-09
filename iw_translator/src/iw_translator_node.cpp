/// \file iw_translator_node.cpp A standalone node for IWTranslator.
///
/// Maps the private namespace of IWTranslator to "solver_model" so that it can
/// be more easily remapped to a place such as "/hbba/solver_model".
///
#include <iw_translator/iw_translator.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iw_translator");

    ros::NodeHandle n, np("solver_model");
    iw_translator::IWTranslator node(n, np);

    ros::spin();
    return 0;
}

