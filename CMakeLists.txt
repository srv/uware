cmake_minimum_required(VERSION 2.8.3)
project(uware)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
  sensor_msgs
  geometry_msgs
  image_transport
  message_filters
  message_generation
  cv_bridge
  image_geometry
  pcl_ros
  cmake_modules
  tf_conversions
  libhaloc
  orb_utils)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")


####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - Eigen:
find_package(Eigen3 REQUIRED)

# Dependencies - PCL:
find_package(PCL REQUIRED)

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})

# Dependencies - Suitesparse:
find_package(SuiteSparse REQUIRED)

# Dependencies - G2O:
# Use script to find g2o. If it's not installed, set G2O_INCLUDE_DIRS manually
# using -DG2O_INCLUDE_DIRS.
find_package(G2O REQUIRED)

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS})

# Pre-process
add_executable(preprocess
  src/nodes/preprocess_node.cpp
  src/preprocess.cpp)
target_link_libraries(preprocess
  ${eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES})

# Reconstruction
add_executable(reconstruction
  src/nodes/reconstruction_node.cpp
  src/registration.cpp
  src/frame_to_frame.cpp
  src/loop_closing.cpp
  src/graph.cpp
  src/build3d.cpp)
target_link_libraries(reconstruction
  ${eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${G2O_LIBRARIES}
  cholmod)