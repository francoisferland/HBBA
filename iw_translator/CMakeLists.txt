cmake_minimum_required(VERSION 2.8.3)
project(iw_translator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  hbba_msgs
  script_engine
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


# This is done to detect if we're building outside of a catkin workspace.
# If we're in one, the or-tools files will be copied in the devel space.
# Otherwise, they will end up in sub-folders of the project directory (include/
# and lib/).
if    (CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(ORTOOLS_OUTPUT_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/../)
else  (CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(ORTOOLS_OUTPUT_DIR ${PROJECT_SOURCE_DIR})
endif (CMAKE_LIBRARY_OUTPUT_DIRECTORY)

set(google-or-tools-ROOT
    ${CMAKE_CURRENT_BINARY_DIR}/google-or-tools-prefix/src/google-or-tools)
include(ExternalProject)
externalproject_add(google-or-tools
    #SVN_REPOSITORY    http://or-tools.googlecode.com/svn/trunk/
    #SVN_REVISION      -r3867
    #SVN_TRUST_CERT    1
    PATCH_COMMAND     patch -p0 -i ${PROJECT_SOURCE_DIR}/or_tools/svn_cert_patch.diff
    GIT_REPOSITORY    https://github.com/google/or-tools.git
    GIT_TAG           v2015-09
    CONFIGURE_COMMAND ""
    BUILD_COMMAND     ${PROJECT_SOURCE_DIR}/or_tools/build.sh
    BUILD_IN_SOURCE   1
    INSTALL_COMMAND   ${PROJECT_SOURCE_DIR}/or_tools/install.sh 
                          ${ORTOOLS_OUTPUT_DIR}
)

set(ORTOOLS_LIBS
    constraint_solver 
    linear_solver
    CoinUtils
    OsiClp
    Osi
    Clp
    Cbc
    CbcSolver
    Cgl
    util
    base
    protobuf
    gflags
    glop
    sat
    split
    bop
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   manyears_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES audio_tools
   CATKIN_DEPENDS roscpp hbba_msgs script_engine
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${ORTOOLS_OUTPUT_DIR}/include/or-tools
)
link_directories(${ORTOOLS_OUTPUT_DIR}/lib/or-tools)

add_library(solver_impl
    src/solver.cpp
)
target_link_libraries(solver_impl
    ${catkin_LIBRARIES}
    ${ORTOOLS_LIBS}
)
# Disables deprecated warnings on or_tools headers and enable C++11 support:
if (CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
                    OUTPUT_VARIABLE GXX_VERSION
    )
    if (NOT GXX_VERSION VERSION_GREATER 4.6)
        message("Trying to force g++-4.8, default is lower than 4.7...")
        set(CMAKE_CXX_COMPILER "g++-4.8")
    endif ()
endif (CMAKE_COMPILER_IS_GNUCXX)

set_target_properties(solver_impl PROPERTIES
    COMPILE_FLAGS "-Wno-deprecated --std=c++11"
)
add_dependencies(solver_impl google-or-tools)

add_library(iw_translator
    src/iw_translator.cpp
    src/solver_model.cpp
    src/strategy_parser.cpp
)
target_link_libraries(iw_translator solver_impl ${catkin_LIBRARIES})

add_executable(iw_translator_node src/iw_translator_node.cpp)
target_link_libraries(iw_translator_node iw_translator ${catkin_LIBRARIES})

add_executable(test_strategy_parser
    tests/test_strategy_parser.cpp
)
target_link_libraries(test_strategy_parser iw_translator ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(iw_translator hbba_msgs_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS audio_tools audio_tools_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_audio_tools.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
