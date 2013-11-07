#include <iw_observer/rules_ast.hpp>
#include "rules_parser.hpp"

#include <ros/ros.h>

extern int yyparse();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rules_compiler");

    yyparse();

    return 0;
}

