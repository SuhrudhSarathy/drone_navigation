cmake_minimum_required(VERSION 3.1.0)
project(drone_navigation)

add_definitions(-std=c++14)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin REQUIRED COMPONENTS
  mav_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  eigen_checks
  mav_trajectory_generation
  mav_trajectory_generation_ros
  mav_planning_msgs
  eigen_conversions
  voxblox
  voxblox_ros
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_service_files(
   FILES
   PointQuery.srv
   Planner.srv
   TrajectoryQuery.srv
   PathOptimiser.srv
)

generate_messages(
   DEPENDENCIES
   mav_msgs
   nav_msgs
   std_msgs  
)

catkin_package(
    INCLUDE_DIRS
     include
     ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
     rrt
     trajectory_optimiser
     voxblox_perception
     commander
    CATKIN_DEPENDS
     mav_msgs
     nav_msgs
     roscpp
     rospy
     std_msgs
     geometry_msgs
     voxblox
     voxblox_ros
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

## LIBRARIES
add_library(rrt
   src/planners/rrt.cpp
)
target_compile_features(rrt INTERFACE cxx_std_11)

add_library(voxblox_perception
   src/voxblox_perception.cpp
)
target_compile_features(voxblox_perception INTERFACE cxx_std_11)

add_library(trajectory_optimiser
   src/trajectory_optimiser.cpp
)
target_compile_features(trajectory_optimiser INTERFACE cxx_std_11)

add_library(commander
   src/commander.cpp
)
target_compile_features(commander INTERFACE cxx_std_11)

add_executable(commander_node 
   src/nodes/commander_node.cpp
)
target_compile_features(commander_node INTERFACE cxx_std_11)

add_dependencies(rrt ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_optimiser ${catkin_EXPORTED_TARGETS})
add_dependencies(voxblox_perception ${catkin_EXPORTED_TARGETS})
add_dependencies(commander ${catkin_EXPORTED_TARGETS})

add_dependencies(commander_node ${catkin_EXPORTED_TARGETS})

set(project_LIBRARIES commander rrt voxblox_perception trajectory_optimiser)

target_link_libraries(rrt ${catkin_LIBRARIES})
target_link_libraries(voxblox_perception ${catkin_LIBRARIES})
target_link_libraries(trajectory_optimiser ${catkin_LIBRARIES})
target_link_libraries(commander ${catkin_LIBRARIES})
target_link_libraries(commander_node ${project_LIBRARIES} ${catkin_LIBRARIES})

install(
  TARGETS commander rrt voxblox_perception trajectory_optimiser
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS commander_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
