cmake_minimum_required(VERSION 2.8.3)
include(CMakeParseArguments)

# add_hbba_cfg(BASE_NAME SRC [BHVR] [OLDREV])
# Automatic build of hbba_cfg with "add_hbba_cfg" macro.
# The BASENAME parameter is the target name and has to be followed by a single
# root source file for your setup.
# The BHVR argument indicates that the configuration should be generated in pure
# behavior-based mode.
# The OLDREV argument activates code generation in the old revision of
# hbba_synth (unsupported).

macro(add_hbba_cfg BASENAME SRC)
    set(_options OLDREV BHVR)
    cmake_parse_arguments(ARG "${_options}" "" "" ${ARGN}) 
    set(HBBA_CFG_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/hbba_cfg_out)
    file(MAKE_DIRECTORY ${HBBA_CFG_OUTPUT_PATH})
    set(HBBA_CFG_OUTPUT_BASENAME
        ${HBBA_CFG_OUTPUT_PATH}/${BASENAME}
    )
    set(HBBA_CFG_OUTPUT
        ${HBBA_CFG_OUTPUT_BASENAME}.launch
        ${HBBA_CFG_OUTPUT_BASENAME}.py
    )
    find_file(_src_path
        ${SRC}
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    message("expanded_src: ${_src_path}") 
#   find_file(HBBA_ROBOT_SRC 
#       ${ROBOT}.yaml 
#       ${PROJECT_SOURCE_DIR}/hbba_cfg 
#       ${irl1_hbba_cfg_PACKAGE_PATH}/hbba_cfg
#   )
    set(HBBA_CFG_SRC_ALL
        ${_src_path}
    )

    set(HBBA_CFG_OPTS "-p")
    # ARGN requires to be put in a variable before list(...) works:
    set(_vargs ${ARGN})

    list(FIND _vargs "BHVR" _opt_bhvr)
    if(NOT _opt_bhvr EQUAL -1)
        message("Building ${BASENAME} in behavior mode.")
        set(HBBA_CFG_OPTS "${HBBA_CFG_OPTS}b")
    endif()

    list(FIND _vargs "OLDREV" _opt_oldrev)
    if (_opt_oldrev EQUAL -1)
        message("Building ${BASENAME} with the new IW revision.")
        set(HBBA_CFG_OPTS "${HBBA_CFG_OPTS}n")
    endif()

    set(HBBA_CFG_OPTS "${HBBA_CFG_OPTS}o")

    message("Gathering HBBA dependencies for ${BASENAME}...")
    execute_process(
        COMMAND rosrun hbba_synth hbba_synth_deps.sh ${HBBA_CFG_SRC}
        OUTPUT_VARIABLE HBBA_CFG_SRC_DEPS
    )
    #message("${BASENAME} dependencies: ${HBBA_CFG_SRC_DEPS}")

    add_custom_command(
        OUTPUT ${HBBA_CFG_OUTPUT}
        COMMAND rosrun hbba_synth hbba_synth ${HBBA_CFG_OPTS} ${HBBA_CFG_OUTPUT_BASENAME} ${HBBA_CFG_SRC_ALL}
        DEPENDS ${HBBA_CFG_SRC_ALL} ${HBBA_CFG_SRC_DEPS}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
    add_custom_target(${BASENAME} ALL DEPENDS ${HBBA_CFG_OUTPUT})
endmacro(add_hbba_cfg)

