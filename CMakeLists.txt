cmake_minimum_required(VERSION 3.5)
project(chargepal_behaviour_tree)

set(CMAKE_CXX_STANDARD 17)
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
  chargepal_services
  roslib
)

find_package(yaml-cpp REQUIRED)

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
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} "util/util.cpp" "scripts/chargepal_bt.cpp")


add_dependencies(
  ${PROJECT_NAME}
  ${chargepal_behaviour_tree_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  SQLiteCpp
  )

find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
    find_package(behaviortree_cpp REQUIRED)
    ament_target_dependencies(${PROJECT_NAME} behaviortree_cpp)

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)
    find_package(catkin REQUIRED COMPONENTS behaviortree_cpp)
    catkin_package(CATKIN_DEPENDS behaviortree_cpp)
    target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS})

    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

else()
    find_package(behaviortree_cpp REQUIRED)
    target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp)

endif()

