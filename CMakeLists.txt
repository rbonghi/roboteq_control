cmake_minimum_required(VERSION 2.8.3)
project(roboteq_control)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

#########################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
#########################################################

## Find catkin macros and librariess
find_package(catkin REQUIRED
                COMPONENTS
                    controller_manager
                    hardware_interface
                    diagnostic_updater
                    roslaunch
                    serial
                    roscpp
                    sensor_msgs
                    std_srvs
                    std_msgs
                    dynamic_reconfigure
                    joint_limits_interface
                    genmsg
                    urdf
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED
                COMPONENTS
                    chrono
                    system
                    thread
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    ControlStatus.msg
    MotorStatus.msg
    Peripheral.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Service.srv
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/RoboteqController.cfg
    cfg/RoboteqParameter.cfg
    cfg/RoboteqEncoder.cfg
    cfg/RoboteqPIDtype.cfg
    cfg/RoboteqPID.cfg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        diagnostic_updater
        std_msgs
        std_srvs
        dynamic_reconfigure
        hardware_interface
        roscpp
        sensor_msgs
        joint_limits_interface
    DEPENDS
        Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/roboteq_control.cpp
# )

set(roboteq_control_SRC
  src/roboteq_control.cpp
  src/roboteq/serial_controller.cpp
  src/roboteq/roboteq.cpp
  src/roboteq/motor.cpp
  src/configurator/motor_param.cpp
  src/configurator/motor_pid.cpp
)
# Include mbs script generator
add_subdirectory(mbs)

# Declare a cpp executable
add_executable(${PROJECT_NAME}_node ${roboteq_control_SRC})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_script ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_script ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME driver_node PREFIX "")

## Declare a cpp executable
#add_executable(roboteq_node ${roboteq_control_SRC})
#target_link_libraries(roboteq_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(roboteq_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".git" EXCLUDE
 )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_roboteq_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
