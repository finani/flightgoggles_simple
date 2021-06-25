cmake_minimum_required(VERSION 3.0)
project(flightgoggles_simple)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Default to building in release mode when no options are set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

################
# USER SETTINGS
################

# Turn this on to compile ROS bindings.
set(COMPILE_ROSCLIENT ON)
set(COMPILE_CPPCLIENT ON)

# Find OMP
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Find OpenCV
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Init Catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mav_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  cv_bridge
  image_transport
)
include_directories(${catkin_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
  LIBRARIES flightgoggles_simple
  CATKIN_DEPENDS
  roscpp
  std_msgs
  mav_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  cv_bridge
  image_transport
)

add_subdirectory(src)

###########
## Build ##
###########

## Declare a C++ library
add_library(inertialMeasurementSim libs/multicopterDynamicsSim/inertialMeasurementSim.cpp)
add_library(multicopterDynamicsSim libs/multicopterDynamicsSim/multicopterDynamicsSim.cpp)
add_library(uavDynamics libs/uavTools/uavDynamics.cpp)
add_library(uavLPF libs/uavTools/uavLPF.cpp)
add_library(uavPID libs/uavTools/uavPID.cpp)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/flightgoggles_simple_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

set_property(DIRECTORY PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/."
  )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(multicopterDynamicsSim
  inertialMeasurementSim
)
target_link_libraries(uavDynamics
  ${catkin_LIBRARIES}
  multicopterDynamicsSim
  uavLPF
  uavPID
)
target_link_libraries(${PROJECT_NAME}_node
  uavDynamics
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)
