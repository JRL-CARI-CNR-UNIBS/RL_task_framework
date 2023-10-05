cmake_minimum_required(VERSION 3.0.2)
project(skills_util)
add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  rosparam_utilities
  skills_executer_msgs
  skills_arbitrator_msgs
  skills_learning_msgs
  skills_util_msgs
  configuration_msgs
  behaviortree_cpp
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp
                rospy
                std_msgs
                rosparam_utilities
 DEPENDS
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/util_functions.cpp
  src/${PROJECT_NAME}/log.cpp
  src/${PROJECT_NAME}/bt_skills_classes.cpp
  src/${PROJECT_NAME}/bt_exec.cpp
  src/${PROJECT_NAME}/configurations_bridge.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

add_executable(        configurations_bridge_node src/configurations_bridge_node.cpp)
add_dependencies(      configurations_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( configurations_bridge_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)

add_executable(        bt_exec_node src/bt_exec_node.cpp)
add_dependencies(      bt_exec_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( bt_exec_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)
