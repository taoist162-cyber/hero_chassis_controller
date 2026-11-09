cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  control_msgs control_toolbox controller_interface hardware_interface
  pluginlib realtime_tools roscpp std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS control_msgs control_toolbox controller_interface hardware_interface
    pluginlib realtime_tools roscpp std_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/hero_chassis_controller.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch doc DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES hero_chassis_controller_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
