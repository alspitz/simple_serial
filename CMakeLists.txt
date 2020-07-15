cmake_minimum_required(VERSION 2.8.3)
project(simple_serial)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  robot_msgs
  roscpp
  geometry_utils
  multirotor_control
  parameter_utils
  nav_msgs
  quadrotor_msgs
  quadrotor_srvs
)

add_message_files(
  FILES
  IMUDebug.msg
  FileRequest.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_utils
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_definitions(
  ${catkin_DEFINITIONS}
)

add_library(${PROJECT_NAME}
  src/SimpleSerial.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  quadrotor_msgs_generate_messages_cpp
  quadrotor_srvs_generate_messages_cpp
  ${PROJECT_NAME}_generate_messages_cpp
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}.cpp
)

set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
set_property(TARGET ${PROJECT_NAME}_node PROPERTY CXX_STANDARD 11)

# add_executable(test_${PROJECT_NAME}
#   src/test_${PROJECT_NAME}.cc
# )
# target_link_libraries(test_${PROJECT_NAME}
#   ${PROJECT_NAME}
# )
# set_property(TARGET test_${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

# add_executable(send_odometry_array
#   src/send_odometry_array.cc
# )
# target_link_libraries(send_odometry_array
#   ${PROJECT_NAME}
# )
# set_property(TARGET send_odometry_array PROPERTY CXX_STANDARD 11)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
