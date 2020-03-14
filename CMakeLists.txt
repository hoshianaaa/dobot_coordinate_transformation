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
  tf
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/config.cfg
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
  ${OpenCV_LIBRARIES}
)

add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(color_detection src/color_detection.cpp)
add_dependencies(color_detection dobot_coordinate_transformer_gencfg) #(ROSパッケージ名)_gencfgという名前で追加
target_link_libraries(color_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

