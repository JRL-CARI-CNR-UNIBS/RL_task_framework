cmake_minimum_required(VERSION 3.0.2)
project(skills_learning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  skills_learning_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
                 std_msgs
                 skills_learning_msgs
  DEPENDS
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/skills_learning/skills_learn.cpp
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

add_executable(        skills_learn_node src/skills_learn_node.cpp)
add_dependencies(      skills_learn_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( skills_learn_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)
