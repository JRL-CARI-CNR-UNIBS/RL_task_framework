cmake_minimum_required(VERSION 3.0.2)
project(skills_executer_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_service_files(
  FILES
  SkillExecution.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)
