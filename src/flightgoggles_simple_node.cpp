/**
 * @file flightgoggles_simple_node.cpp
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Implementation of UAV dynamics, IMU, and angular rate control simulation node
 *
 */

#include "../libs/uavTools/uavDynamics.h"

/**
 * @brief Global function designated start of UAV dynamics node
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "flightgoggles_simple_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Init class
  UavDynamics uav_dynamics_node(n);

  // Spin
  ros::spin();

  return 0;
}
