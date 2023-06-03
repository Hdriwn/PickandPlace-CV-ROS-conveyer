cmake_minimum_required(VERSION 3.1.3)
project(sliding_pickplace)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs  # Add geometry_msgs as a dependency
)

# Generate the UserRequest.h file
add_service_files(
  FILES
  UserRequest.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs  # Add geometry_msgs as a dependency
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  message_runtime
  geometry_msgs  # Add geometry_msgs as a dependency
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(slider scripts/slidepickandplace.cpp)
target_link_libraries(slider ${catkin_LIBRARIES})

add_executable(prismat scripts/prismat.cpp)
target_link_libraries(prismat ${catkin_LIBRARIES})

add_executable(pickup scripts/pickandplace.cpp)
target_link_libraries(pickup ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

