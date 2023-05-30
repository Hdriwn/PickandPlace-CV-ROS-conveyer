cmake_minimum_required(VERSION 3.1.3)
project(pickplacev)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  cv_bridge
  image_transport
  sensor_msgs
  tf
  tf2_ros
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pickandplace scripts/pickandplace.cpp)
target_link_libraries(pickandplace
  ${catkin_LIBRARIES}
)

add_executable(pickandplace_cc scripts/pickandplace_cc.cpp)
target_link_libraries(pickandplace_cc
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)



install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
