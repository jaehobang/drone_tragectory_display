cmake_minimum_required(VERSION 2.8.3)
project(joint_mapping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(GTSAM REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(flirtlib REQUIRED)
find_package(Boost 1.54 REQUIRED)

find_package(PCL 1.4 REQUIRED COMPONENTS
  common
  roscpp
  kdtree
  filters
  features
  keypoints
  sample_consensus
  registration
  segmentation
  visualization
  pcl_conversions
)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "This program requires the PCL library, and will not be compiled.")
endif(NOT PCL_FOUND)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  geometry_msgs
  message_generation
  parameter_utils
  dense_mapping
  sensor_msgs
  nav_msgs
  asio_udp_device
  pcl_conversions
  pcl_ros
)

find_package(Eigen REQUIRED)

add_message_files(
  FILES
  RelativePose.msg
  AgentPackageMsg.msg
  PointCloudWithPose.msg
  MatchPointClouds.msg
  TimedTask.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp message_runtime nav_msgs sensor_msgs std_msgs
  DEPENDS
    GTSAM
    Eigen
    PCL
    message_runtime
    pcl_conversions
    flirtlib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)
 # ${Boost_INCLUDE_DIRS}
message(STATUS "Found PCL and GTSAM")

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${catkin_DEFINIONS}
  ${PCL_DEFINITIONS}
)

add_library(${PROJECT_NAME}
  src/Clustering3D.cpp
  src/CRProcess.cpp
  src/Hypothesis.cpp
  src/LocalAgent.cpp
  src/ForeignAgent.cpp
  src/Matcher3D.cpp
  src/Package.cpp
  src/Network.cpp
  src/RelativePoseEM.cpp
  src/SmallEM.cpp
  src/Utils.cpp
  src/RobotManager.cpp
  src/asio_udp_utils.cc
  src/Logger.cpp
  src/Matcher2D.cpp
  src/ICPMatcher2D.cpp
)

message(STATUS "flirtlib: ${flirtlib_LIBRARIES}")

add_executable(${PROJECT_NAME}_node
  src/joint_mapping.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${libpointmatcher_LIBRARIES}
  ${flirtlib_LIBRARIES}
  gtsam
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
