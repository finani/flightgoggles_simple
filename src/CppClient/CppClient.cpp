/**
 * @file   CppClient.cpp
 * @author Winter Guerra
 * @brief  Pulls images from Unity and saves them as PNGs
 *space.
 **/

#include "CppClient.hpp"

#define SHOW_DEBUG_IMAGE_FEED false

/**
 * CppClient constructor
 */
CppClient::CppClient()
{
  startTime = flightGoggles.getTimestamp();
}

/**
 * imageConsumer thread to consume images from FG
 * @param self General client object
 */
void imageConsumer(CppClient *self)
{
  while (true)
  {
    // Wait for render result (blocking).
    unity_incoming::RenderOutput_t renderOutput = self->flightGoggles.handleImageResponse();

    // Display result
    if (SHOW_DEBUG_IMAGE_FEED)
    {
      cv::imshow("Debug RGB", renderOutput.images[0]);
      cv::imshow("Debug D", renderOutput.images[1]);
      cv::waitKey(1);
    }
  }
}

/**
 * posePublisher publish the current pose to FG
 * @param self General client object
 */
void posePublisher(CppClient *self)
{
  // Sends render requests to FlightGoggles indefinitely
  while (true)
  {
    // Update camera position
    self->updateCameraTrajectory();
    // Update timestamp of state message (needed to force FlightGoggles to rerender scene)
    self->flightGoggles.state.ntime = self->flightGoggles.getTimestamp();
    // request render
    self->flightGoggles.requestRender();
    // Throttle requests to 60hz.
    usleep(1e6 / 60.0f);
  }
}

/**
 * addCameras function to add camera objects to the client
 */
void CppClient::addCameras()
{
  // Prepopulate metadata of cameras (RGBD)
  unity_outgoing::Camera_t cam_Left;
  cam_Left.ID = "Camera_Left";

  // unity_outgoing::Camera_t cam_Right;
  // cam_Right.ID = "Camera_Right";

  //  unity_outgoing::Camera_t cam_D;
  //  cam_D.ID = "Camera_D";

  // Add cameras to persistent state
  flightGoggles.state.cameras.push_back(cam_Left);
  // flightGoggles.state.cameras.push_back(cam_Right);
}

/**
 * updateCameraTrajectory function to generate a simple circular trajectory
 */
void CppClient::updateCameraTrajectory()
{
  double period = 15.0f;
  double r = 1.5f;
  double t = (flightGoggles.getTimestamp() - startTime) / 1000000.0f;
  double theta = -((t / period) * 2.0f * M_PI);

  Transform3 camera_pose;
  camera_pose.translation() = Vector3(r * cos(theta), r * sin(theta), -1.0f);
  // Set rotation matrix using pitch, roll, yaw
  camera_pose.linear() = Eigen::AngleAxisd(theta - M_PI, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

  // Populate status message with new pose
  flightGoggles.setCameraPoseUsingROSCoordinates(camera_pose, 0);
  // flightGoggles.setCameraPoseUsingROSCoordinates(camera_pose, 1);
}

int main()
{
  // Create client
  CppClient cppClient;

  // Instantiate RGBD cameras
  cppClient.addCameras();

  // Set scene parameters.
  /*
  Available scenes: [
    "Hazelwood_Loft_Full_Night"
    "Hazelwood_Loft_Full_Day",
    "Butterfly_World",
    "FPS_Warehouse_Day",
    "FPS_Warehouse_Night",
  ]
   */
  cppClient.flightGoggles.state.sceneFilename = "Hazelwood_Loft_Full_Night";

  // Fork sample render request thread
  // will request a simple circular trajectory
  std::thread posePublisherThread(posePublisher, &cppClient);

  // Fork a sample image consumer thread
  std::thread imageConsumerThread(imageConsumer, &cppClient);

  // Spin
  while (true)
  {
    sleep(1);
  }

  return 0;
}
