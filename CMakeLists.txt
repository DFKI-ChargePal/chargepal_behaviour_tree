cmake_minimum_required(VERSION 3.0.2)
project(chargepal_behaviour_tree)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  chargepal_actions
  roslib
)

find_package(behaviortree_cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    actionlib
    actionlib_msgs
    roslib
)

add_subdirectory(SQLiteCpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${behaviortree_cpp_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} "scripts/chargepal_bt.cpp" "util/util.cpp")

add_dependencies(
  ${PROJECT_NAME}
  ${chargepal_behaviour_tree_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  BT::behaviortree_cpp
  ${catkin_LIBRARIES}
  SQLiteCpp
  )

