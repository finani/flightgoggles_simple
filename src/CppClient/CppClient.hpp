#ifndef CPPCLIENT_H
#define CPPCLIENT_H
/**
 * @file   CppClient.hpp
 * @author Winter Guerra
 * @brief  Basic client interface for FlightGoggles.
 */

#include <../Common/FlightGogglesClient.hpp>

#include <unistd.h>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

class CppClient
{
public:
  ///  FlightGoggles interface object
  FlightGogglesClient flightGoggles;

  /// Counter for keeping track of trajectory position
  int64_t startTime;

  /// constructor
  CppClient();

  /// Add RGBD camera settings to scene.
  void addCameras();

  /// Do a simple trajectory with the camera
  void updateCameraTrajectory();
};

#endif
