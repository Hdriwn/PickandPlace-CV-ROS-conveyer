cmake_minimum_required(VERSION 2.8.3)
project(robotiq_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  control_toolbox
  xmlrpcpp
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    control_toolbox
    xmlrpcpp
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Add any additional libraries or executables if needed


