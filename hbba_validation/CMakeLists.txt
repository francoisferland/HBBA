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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

add_executable(state_machine_test
    src/state_machine_test.cpp
)

rosbuild_add_library(${PROJECT_NAME}
    src/landmarks_observer.cpp
    src/goto_landmark.cpp
    src/landmarks_interaction.cpp
    src/cardreader_localizer.cpp
    src/cardreader_localizer_openni.cpp
    src/cardreader_localizer_imv.cpp
    src/unlock_door.cpp
    src/tour_guide.cpp
    src/turn_around.cpp
    src/opt_flow_pose.cpp
)

rosbuild_add_executable(goto_landmark_node
    src/goto_landmark_node.cpp
)
target_link_libraries(goto_landmark_node ${PROJECT_NAME})

rosbuild_add_executable(landmarks_interaction_node
    src/landmarks_interaction_node.cpp
)
target_link_libraries(landmarks_interaction_node ${PROJECT_NAME})

rosbuild_add_executable(cardreader_localizer_test
    src/cardreader_localizer_test.cpp
)
target_link_libraries(cardreader_localizer_test ${PROJECT_NAME})

rosbuild_add_executable(unlock_door_node
    src/unlock_door_node.cpp
)
target_link_libraries(unlock_door_node ${PROJECT_NAME})

rosbuild_add_executable(tour_guide_node
    src/tour_guide_node.cpp
)
target_link_libraries(tour_guide_node ${PROJECT_NAME})

rosbuild_add_executable(turn_around_node
    src/turn_around_node.cpp
)
target_link_libraries(turn_around_node ${PROJECT_NAME})

rosbuild_add_executable(opt_flow_pose_node
    src/opt_flow_pose_node.cpp
)
target_link_libraries(opt_flow_pose_node ${PROJECT_NAME})

# Automatic build of hbba_cfg:
rosbuild_find_ros_package(hbba_synth)
include(${hbba_synth_PACKAGE_PATH}/cmake/build.cmake)

add_hbba_cfg(scenario         irl1_tr NEWREV)
add_hbba_cfg(tour_guide       irl1_tr NEWREV)
add_hbba_cfg(test_landmarks   irl1_tr NEWREV)
add_hbba_cfg(test_unlock_door irl1_tr NEWREV)

