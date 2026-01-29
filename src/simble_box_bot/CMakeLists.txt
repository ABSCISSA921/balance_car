cmake_minimum_required(VERSION 3.0.2)
project(simble_box_bot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(Eigen3 REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  sensor_msgs
  realtime_tools
  tf
  dynamic_reconfigure
  control_toolbox
  geometry_msgs
  cmake_modules
)

generate_dynamic_reconfigure_options(
  cfg/BalanceParam.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simble_box_bot
#  CATKIN_DEPENDS controller_manager gazebo_ros robot_state_publisher urdf xacro
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_library(balance_controller 
 src/balance_controller.cpp 
 src/lqr_controller.cpp)
add_dependencies(balance_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(balance_controller ${catkin_LIBRARIES})

install(TARGETS balance_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES balance_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

