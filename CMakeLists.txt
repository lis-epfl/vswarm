cmake_minimum_required(VERSION 2.8.3)
project(vswarm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  vision_msgs
  vrpn_client_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/FlockingNode.cfg
  cfg/MigrationNode.cfg
  cfg/MultiTargetTrackingNode.cfg
  cfg/ObjectDetectionNode.cfg
  cfg/RelativeLocalizationNode.cfg
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
)

add_executable(gazebo_mocap_node src/gazebo_mocap_node.cpp)
target_link_libraries(gazebo_mocap_node ${catkin_LIBRARIES})
