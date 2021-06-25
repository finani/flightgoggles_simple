/**
 * @file uavDynamics.h
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Header file for UAV dynamics, IMU, and angular rate control simulation node
 *
 */

#ifndef UAV_DYNAMICS_HPP
#define UAV_DYNAMICS_HPP

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

// Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/RateThrust.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include "../multicopterDynamicsSim/multicopterDynamicsSim.hpp"
#include "uavLPF.h"
#include "uavPID.h"

#include <random>
#include <sstream>

/**
 * @brief UAV Dynamics class used for dynamics, IMU, and angular rate control simulation node
 *
 */
class UavDynamics
{
public:
  /// @name Constructor
  UavDynamics(ros::NodeHandle nh);

  /// @name Node handle
  ros::NodeHandle node_;

  /// @name Transform Publishers
  //@{
  tf2_ros::TransformBroadcaster tfPub_;
  tf2_ros::StaticTransformBroadcaster staticTfPub_;
  //@}

  /// @name Publishers
  //@{
  ros::Publisher imuPub_;
  ros::Publisher odomPub_;
  ros::Publisher clockPub_;

  void publishState(void);
  void publishIMUMeasurement(void);
  void publishStaticMotorTransform(const ros::Time& timeStamp, const char* frame_id, const char* child_frame_id,
                                   const Eigen::Isometry3d& motorFrame);
  //@}

  /// @name Subscribers
  //@{
  ros::Subscriber inputCommandSub_;
  ros::Subscriber inputMotorspeedCommandSub_;
  ros::Subscriber collisionSub_;
  ros::Subscriber armSub_;
  ros::Subscriber resetSub_;
  ros::Subscriber frameRateSub_;
  //@}

  /// @name Timers
  //@{
  ros::WallTimer simulationLoopTimer_;
  //@}

  /// @name Callbacks
  //@{
  void simulationLoopTimerCallback(const ros::WallTimerEvent& event);
  void inputCallback(mav_msgs::RateThrust::Ptr msg);
  void inputMotorspeedCallback(mav_msgs::Actuators::Ptr msg);
  void armCallback(std_msgs::Empty::Ptr msg);
  void resetCallback(std_msgs::Empty::Ptr msg);
  void collisionCallback(std_msgs::Empty::Ptr msg);
  void fpsCallback(std_msgs::Float32::Ptr msg);
  //@}

  /// @name Control inputs
  //@{
  mav_msgs::RateThrust::Ptr lastCommandMsg_;
  mav_msgs::Actuators::Ptr lastMotorspeedCommandMsg_;
  //@}

  /// @name Time keeping variables
  //@{
  ros::Time currentTime_;
  double dt_secs = 1.0f / 960.;
  bool useAutomaticClockscale_ = false;
  double clockScale = 1.0;
  double actualFps = -1;
  bool useSimTime_ = false;
  //@}

  /// @name Simulation state control parameters and flags
  //@{
  bool hasCollided_ = false;
  bool ignoreCollisions_ = false;
  bool armed_ = false;
  bool resetRequested_ = false;
  bool useRateThrustController_ = true;
  bool useRungeKutta4Integrator_ = false;
  //@}

  void resetState(void);

  /// @name Angular rate control PID controller and LPF
  //@{
  UavPID pid_;
  UavLPF lpf_;
  //@}

  /// @name Vehicle dynamics simulator
  //@{
  /* As standard in ROS:
  x : forward
  y : leftward
  z : upward */

  /* Motors are numbered counterclockwise (look at quadcopter from above) with
     motor 1 in the positive quadrant of the X-Y plane (i.e. front left). */

  MulticopterDynamicsSim* multicopterSim_;
  //@}

  /// @name Initial conditions
  //@{
  Eigen::Vector3d initPosition_;
  Eigen::Quaterniond initAttitude_;
  double initPropSpeed_;
  //@}

  /// @name IMU measurements and variances
  //@{
  Eigen::Vector3d imuAccOutput_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d imuGyroOutput_ = Eigen::Vector3d::Zero();
  double gyroMeasNoiseVariance_;
  double accMeasNoiseVariance_;
  double accBiasInitVar_;
  double gyroBiasInitVar_;
  //@}
};

#endif
