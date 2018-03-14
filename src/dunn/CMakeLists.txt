cmake_minimum_required(VERSION 2.8.3)
project(dunn)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
