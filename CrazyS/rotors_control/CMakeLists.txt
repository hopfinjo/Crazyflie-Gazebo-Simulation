cmake_minimum_required(VERSION 3.0.2)
project(rotors_control VERSION 7.0.1)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES position_controller crazyflie_onboard_controller
   sensfusion6 crazyflie_complementary_filter 
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(position_controller
   src/library/position_controller.cpp
)

add_library(attitude_controller
   src/library/attitude_controller.cpp
)

add_library(position_controller_v2
   src/library/position_controller_v2.cpp
)

add_library(attitude_controller_v2
   src/library/attitude_controller_v2.cpp
)


add_library(crazyflie_onboard_controller
   src/library/crazyflie_onboard_controller.cpp
)

add_library(crazyflie_complementary_filter
  src/library/crazyflie_complementary_filter.cpp
)

add_library(sensfusion6
  src/library/sensfusion6.cpp
)

target_link_libraries(position_controller ${catkin_LIBRARIES})
add_dependencies(position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(attitude_controller ${catkin_LIBRARIES})
add_dependencies(attitude_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(crazyflie_onboard_controller ${catkin_LIBRARIES})
add_dependencies(crazyflie_onboard_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(crazyflie_complementary_filter ${catkin_LIBRARIES})
add_dependencies(crazyflie_complementary_filter ${catkin_EXPORTED_TARGETS})

target_link_libraries(sensfusion6 ${catkin_LIBRARIES})
add_dependencies(sensfusion6 ${catkin_EXPORTED_TARGETS})

add_executable(position_controller_node src/nodes/position_controller_node.cpp)
target_link_libraries(position_controller_node
  position_controller crazyflie_complementary_filter crazyflie_onboard_controller
  sensfusion6 ${catkin_LIBRARIES})
add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS})

add_executable(position_controller_node_v2 src/nodes/position_controller_node_v2.cpp)
target_link_libraries(position_controller_node_v2
  position_controller_v2 crazyflie_complementary_filter crazyflie_onboard_controller
  sensfusion6 ${catkin_LIBRARIES})
add_dependencies(position_controller_node_v2 ${catkin_EXPORTED_TARGETS})

add_executable(attitude_controller_node_v2 src/nodes/attitude_controller_node_v2.cpp)
target_link_libraries(attitude_controller_node_v2
  attitude_controller_v2 ${catkin_LIBRARIES})
add_dependencies(attitude_controller_node_v2 ${catkin_EXPORTED_TARGETS})



add_executable(attitude_controller_node src/nodes/attitude_controller_node.cpp)
target_link_libraries(attitude_controller_node
  attitude_controller ${catkin_LIBRARIES})
add_dependencies(attitude_controller_node ${catkin_EXPORTED_TARGETS})


install(TARGETS position_controller crazyflie_onboard_controller attitude_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS position_controller_node attitude_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)