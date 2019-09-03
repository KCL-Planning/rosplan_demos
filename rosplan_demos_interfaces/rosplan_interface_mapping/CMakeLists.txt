cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_mapping)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosplan_knowledge_msgs
  nav_msgs
  std_srvs
  geometry_msgs
  diagnostic_msgs
  visualization_msgs
  tf
  occupancy_grid_utils
  message_generation
  genmsg
)

add_service_files(
  FILES
  CreatePRM.srv
  AddWaypoint.srv
  RemoveWaypoint.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# waypoint generation server
add_executable(rp_roadmap_server src/RPRoadmapServer.cpp)
add_dependencies(rp_roadmap_server rosplan_interface_mapping_generate_messages_cpp)
target_link_libraries(rp_roadmap_server ${catkin_LIBRARIES})

# waypoint filter
add_executable(rp_roadmap_filter src/RPRoadmapFilter.cpp)
target_link_libraries(rp_roadmap_filter ${catkin_LIBRARIES})

##########
## Test ##
##########

# please do not use add_rosttest_gtest (seems to be interfering with qtcreator and cmake)

#############
## Install ##
#############

install(TARGETS rp_roadmap_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
