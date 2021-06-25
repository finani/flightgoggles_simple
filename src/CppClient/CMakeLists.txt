# Include ROS dirs
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Compile ROS node
add_executable(CppClient CppClient.cpp)
target_link_libraries(CppClient
  FlightGogglesClientLib
  ${OpenCV_LIBS}
  pthread
)
