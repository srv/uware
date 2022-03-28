cmake_minimum_required(VERSION 2.8.3)
project(uware)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   #message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
   set(CMAKE_CXX_STANDARD 14)
   add_definitions(-DCOMPILEDWITHC14)
   message(STATUS "Using CMAKE_CXX_STANDARD")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  cola2_msgs
  ned_tools
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
  ##libhaloc
  orb_utils)

####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - Eigen:
find_package(Eigen3 REQUIRED)

# Dependencies - PCL:
find_package(PCL REQUIRED)

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# Dependencies - Suitesparse:
find_package(SuiteSparse REQUIRED)

# Dependencies - G2O:
# Use script to find g2o. If it's not installed, set G2O_INCLUDE_DIRS manually
# using -DG2O_INCLUDE_DIRS.
## find_package(G2O REQUIRED) # commented by fbf 19/01/2021 . uncomment for SLAM optimization

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uware
  CATKIN_DEPENDS image_transport cv_bridge roscpp sensor_msgs geometry_msgs cola2_msgs ned_tools
  DEPENDS OpenCV
  DEPENDS Eigen3
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}) # this closing braked must go at the end of the next line, if the former is uncommented
  #${G2O_INCLUDE_DIRS}) commented by fbf 19/01/2021 . uncomment for SLAM optimization

# Pre-process
add_executable(preprocess
  src/nodes/preprocess_node.cpp
  src/preprocess.cpp)
target_link_libraries(preprocess
  ${eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES})

# Reconstruction commented by fbf 19/01/2021 . uncomment for 3D reconstruction
##add_executable(reconstruction
##  src/nodes/reconstruction_node.cpp
##  src/registration.cpp
##  src/frame_to_frame.cpp
##  src/loop_closing.cpp
##  src/graph.cpp
##  src/build3d.cpp)
##target_link_libraries(reconstruction
##  ${eigen_LIBRARIES}
##  ${OpenCV_LIBRARIES}
##  ${catkin_LIBRARIES}
##  ${SUITESPARSE_LIBRARIES}
##  ${G2O_LIBRARIES}
##  cholmod)
