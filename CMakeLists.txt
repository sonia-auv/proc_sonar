# \file     CMakeLists.txt
# \author   Marc-Antoine Couture <coumarc9@outlook.com>
# \date     07/14/2019
# \last-update 07/14/2019
# \copyright    2019 Club SONIA AUV, ETS. All rights reserved.
# Use of this source code is governed by the GPL v3 that can be
# found in the LICENSE file.

#===============================================================================
# G L O B A L   P R O J E C T   C O N F I G U R A T I O N

cmake_minimum_required(VERSION 2.8.3)
project(proc_sonar)

if ( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE Debug CACHE STRING
            "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
            FORCE )
endif ( NOT CMAKE_BUILD_TYPE )

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -Wall" )
set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -O3" )

#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package( catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        provider_sonar
        message_generation)



# Declare this project as a catkin package and export the necessary build flags
catkin_package(
        INCLUDE_DIRS src
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        provider_sonar
        message_runtime)

#===============================================================================
# I N C L U D E   F I L E S

set(proc_sonar_INCLUDE_DIRS "src")
set(proc_sonar_SRC_DIR "src/proc_sonar")

# Include all the directories where source can be found.
include_directories(
        ${proc_sonar_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}

)

#==========================================================================
# C R E A T E   E X E C U T A B L E

set( TARGET_NAME proc_sonar_node )
add_executable( ${TARGET_NAME}
        ${proc_sonar_SRC_DIR}/main.cc
        ${proc_sonar_SRC_DIR}/proc_sonar_node.cc)
target_link_libraries( ${TARGET_NAME} ${catkin_LIBRARIES})
set_target_properties( ${TARGET_NAME} PROPERTIES LINKER_LANGUAGE CXX )
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)