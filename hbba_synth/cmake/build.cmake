cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Automatic build of hbba_cfg with "add_hbba_cfg" macro.
# The BASENAME parameter should name a configuration file found in your project
# package's hbba_cfg subfolder.
# The ROBOT parameter should point to a config file basename found in
# "irl1_hbba_cfg", such as "irl1_tr".

rosbuild_find_ros_package(irl1_hbba_cfg)
# Need to find the location of this macro's package for building and
# dependencies tools:
rosbuild_find_ros_package(hbba_synth)

set(HBBA_CFG_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/hbba_cfg_out)
macro(add_hbba_cfg BASENAME ROBOT)
    file(MAKE_DIRECTORY ${HBBA_CFG_OUTPUT_PATH})
    set(HBBA_CFG_OUTPUT_BASENAME
        ${HBBA_CFG_OUTPUT_PATH}/${BASENAME}
    )
    set(HBBA_CFG_OUTPUT
        ${HBBA_CFG_OUTPUT_BASENAME}.launch
        ${HBBA_CFG_OUTPUT_BASENAME}.py
    )
    set(HBBA_CFG_SRC
        ${PROJECT_SOURCE_DIR}/hbba_cfg/${BASENAME}.yaml
    )
    set(HBBA_CFG_SRC_ALL
        ${irl1_hbba_cfg_PACKAGE_PATH}/hbba_cfg/${ROBOT}.yaml
        ${HBBA_CFG_SRC}
    )

    message("Gathering HBBA dependencies for ${BASENAME}...")
    execute_process(
        COMMAND rosrun hbba_synth hbba_synth_deps.sh ${HBBA_CFG_SRC}
        OUTPUT_VARIABLE HBBA_CFG_SRC_DEPS
    )
    #message("${BASENAME} dependencies: ${HBBA_CFG_SRC_DEPS}")

    add_custom_command(
        OUTPUT ${HBBA_CFG_OUTPUT}
        COMMAND rosrun hbba_synth hbba_synth -po ${HBBA_CFG_OUTPUT_BASENAME} ${HBBA_CFG_SRC_ALL}
        DEPENDS ${HBBA_CFG_SRC_ALL} ${HBBA_CFG_SRC_DEPS}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
    add_custom_target(${BASENAME} ALL DEPENDS ${HBBA_CFG_OUTPUT})
endmacro(add_hbba_cfg)

