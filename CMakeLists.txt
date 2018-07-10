cmake_minimum_required(VERSION 2.8.3)
project(tb3_navigation)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tb3_navigation
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(tb3_navigation
         src/tb3_navigation.cpp
         src/pidImpl.h)

 target_link_libraries(tb3_navigation
   ${catkin_LIBRARIES}
 )
