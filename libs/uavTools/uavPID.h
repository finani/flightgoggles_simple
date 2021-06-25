/**
 * @file uavPID.h
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Header file for UAV PID Controller
 *
 */

#ifndef UAV_PID_H
#define UAV_PID_H

// ROS includes
#include <ros/ros.h>

#include <Eigen/Dense>

/**
 * @brief PID controller class used for angular rate control.
 *
 */
class UavPID
{
public:
  /// @name Constructor
  UavPID();

  void controlUpdate(Eigen::Array3d& command, double thrustCommand, Eigen::Array3d& curval, Eigen::Array3d& curder,
                     Eigen::Array4d& propSpeedCommand, double dt);
  void resetState(void);
  void thrustMixing(Eigen::Array4d& propSpeedCommand, Eigen::Array3d& angAccCommand, double thrustCommand);

private:
  /// @name PID Controller Gains
  //@{
  Eigen::Array3d propGain_ = Eigen::Array3d::Zero();
  Eigen::Array3d intGain_ = Eigen::Array3d::Zero();
  Eigen::Array3d derGain_ = Eigen::Array3d::Zero();
  //@}

  /// @name PID Controller Integrator State and Bound
  //@{
  Eigen::Array3d intState_ = Eigen::Array3d::Zero();
  Eigen::Array3d intBound_ = Eigen::Array3d::Zero();
  //@}

  /// @name PID Controller Vehicle Parameters
  //@{
  Eigen::Array3d vehicleInertia_ = Eigen::Array3d::Zero();
  double momentArm_ = 0.08;
  double thrustCoeff_ = 1.91e-6;
  double torqueCoeff_ = 2.6e-7;
  //@}
};

#endif  // UAV_PID_H