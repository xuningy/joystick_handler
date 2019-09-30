cmake_minimum_required(VERSION 2.8.3)
project(joystick_handler)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

find_package(catkin REQUIRED COMPONENTS
      roscpp
      control_arch
      geometry_utils
      joystick_ui
      parameter_utils
      quadrotor_msgs
      one_euro_filter
      std_msgs
)

add_message_files(
  FILES
  JoystickValues.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS
            roscpp
            control_arch
            geometry_utils
            joystick_ui
            parameter_utils
            quadrotor_msgs
            one_euro_filter
            std_msgs
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
      src/JoystickHandler.cpp
)

target_link_libraries(${PROJECT_NAME}
      ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
      src/joystick_handler.cpp
)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

set_target_properties(${PROJECT_NAME}_node
    PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_node
      ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h"
)
