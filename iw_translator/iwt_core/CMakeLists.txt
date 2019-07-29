cmake_minimum_required(VERSION 2.8.3)
project(iwt_core)

find_package(Boost REQUIRED)

# Google or-tools library - we now rely on pre-built packages.
# See or-tools/install.sh script to automate installation.
find_library(ORTOOLS_LIBS 
    ortools
    HINTS /opt/or-tools/lib
)
if(NOT ORTOOLS_LIBS)
    message(FATAL_ERROR "
        Google or-tools lib not found, please install first.
        See install script in ${PROJECT_SOURCE_DIR}/or_tools for details."
    )
endif(NOT ORTOOLS_LIBS)
find_path(ORTOOLS_INCLUDE_DIR 
    ortools/constraint_solver/constraint_solver.h
    HINTS /opt/or-tools/include
)
if(NOT ORTOOLS_INCLUDE_DIR)
    message(FATAL_ERROR "
        Google or-tools headers not found, please install first.
        See install script in ${PROJECT_SOURCE_DIR}/or_tools for details."
    )
endif(NOT ORTOOLS_INCLUDE_DIR)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${ORTOOLS_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

add_library(iwt_core
    src/solver_model.cpp
    src/solver.cpp
)

set_target_properties(iwt_core PROPERTIES
    COMPILE_FLAGS "-Wno-deprecated --std=c++11"
)

target_link_libraries(iwt_core
    ${ORTOOLS_LIBS}
)

install(TARGETS iwt_core
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/iwt_core
    DESTINATION include/
)
