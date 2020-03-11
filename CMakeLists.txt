cmake_minimum_required(VERSION 2.8.3)
project(dobot_coordinate_transformer)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  pcl_ros
  roscpp
  rospy
  rospy
  sensor_msgs
  std_msgs
  dobot_msgs
  message_generation
)

add_service_files(
  FILES
  CameraCoordinate.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dobot_coordinate_transformer
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure geometry_msgs image_transport pcl_ros roscpp rospy rospy sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(coordinate_transformer src/coordinate_transformer.cpp)
target_link_libraries(coordinate_transformer
  ${catkin_LIBRARIES}
)
add_dependencies(coordinate_transformer coordinate_transformer_gencpp)

add_executable(test src/test.cpp)
target_link_libraries(test
  ${catkin_LIBRARIES}
)
