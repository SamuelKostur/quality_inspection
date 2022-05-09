cmake_minimum_required(VERSION 3.0.2)
project(quality_inspection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  actionlib_msgs
  actionlib
  message_generation
  cmake_modules
  pcl_ros
  cv_bridge
  image_transport
  phoxi_camera
)

## System dependencies are found with CMake's conventions
find_package(TinyXML REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
# pcl_ros is supposed to be sufficient so no need to add find_package(PCL1.8 REQUIRED)
# however skipping OpenCV, Eigen3, Boost also works

## Generate services in the 'srv' folder
add_service_files(
  FILES
  create2Dprojections.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES  MovRobToScanPos.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qualityInspection
  DEPENDS message_runtime
  DEPENDS TinyXML
  DEPENDS OpenCV
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS} 
  ${TinyXML_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(robot_communication_node src/robot_communication_node.cpp)
add_executable(main_control_node src/main_control_node.cpp)
add_executable(scan_processing_node src/scan_processing_node.cpp src/scan_processing.cpp)
add_executable(scanning_simulation_node src/scanning_simulation_node.cpp)
add_executable(projections_creation_node src/projections_creation_node.cpp)
add_executable(transform_cad src/transform_cad.cpp)

add_dependencies(robot_communication_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(main_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(scan_processing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(scanning_simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(projections_creation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(transform_cad ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(robot_communication_node
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
target_link_libraries(main_control_node
  ${catkin_LIBRARIES}
)
target_link_libraries(scan_processing_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(scanning_simulation_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(projections_creation_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(transform_cad
  ${catkin_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})