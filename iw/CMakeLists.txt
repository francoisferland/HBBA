cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_library(iw_nodelets src/intention_workspace_nodelet.cpp)
rosbuild_add_executable(iw_cpp src/intention_workspace.cpp)

rosbuild_add_library(iw 
    src/exploitation_matcher.cpp
    src/events_generator.cpp
    src/events_filter.cpp
    src/rosgraph_monitor.cpp
)
rosbuild_add_executable(events_generator src/events_generator_node.cpp)
target_link_libraries(events_generator iw)
rosbuild_add_executable(exploitation_matcher src/exploitation_matcher_node.cpp)
target_link_libraries(exploitation_matcher iw)

rosbuild_add_executable(goto_events_observer src/goto_events_observer.cpp)
target_link_libraries(goto_events_observer iw yaml-cpp)

rosbuild_add_executable(say_events_observer src/say_events_observer.cpp)
target_link_libraries(say_events_observer iw)

rosbuild_add_executable(empty_event_generator src/empty_event_generator.cpp)
