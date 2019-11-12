cmake_minimum_required(VERSION 2.8.12)
project(choreonoid_ros_tank_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  choreonoid
  )

catkin_package(SKIP_CMAKE_CONFIG_GENERATION SKIP_PKG_CONFIG_GENERATION)

if(CHOREONOID_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  set(CMAKE_CXX_EXTENSIONS OFF)
else()
  add_compile_options(-std=c++14)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${CHOREONOID_INCLUDE_DIRS}
  )

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_subdirectory(src)

#install(FILES
#  project/step1.cnoid
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/project)
#
#install(FILES
#  launch/step1.launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
