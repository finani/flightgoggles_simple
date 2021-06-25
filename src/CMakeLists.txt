# Always compile these dirs
add_subdirectory(Common)

# Only compile ROS client if ROS is installed.
if(COMPILE_ROSCLIENT)
  add_subdirectory(ROSClient)
endif()

if(COMPILE_CPPCLIENT)
  add_subdirectory(CppClient)
endif()