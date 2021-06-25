# Include ROS dirs
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Compile ROS node
add_executable(ROSClient ROSClient.cpp)
target_link_libraries(ROSClient
  FlightGogglesClientLib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  pthread
)





