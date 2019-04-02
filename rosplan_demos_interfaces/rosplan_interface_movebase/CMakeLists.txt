cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_movebase)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  diagnostic_msgs
  geometry_msgs
  move_base_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rosplan_planning_system
  actionlib
  tf2
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rpmovebase src/RPMoveBase.cpp)
target_link_libraries(rpmovebase ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS rpmovebase
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
