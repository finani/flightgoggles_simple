/**
 * @file uavLPF.h
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Header file for UAV Low Pass Filter
 *
 */

#ifndef UAV_LPF_H
#define UAV_LPF_H

// ROS includes
#include <ros/ros.h>

#include <Eigen/Dense>

/**
 * @brief Low-pass filter class used for angular rate control.
 *
 */
class UavLPF
{
public:
  /// @name Constructor
  UavLPF();

  void proceedState(Eigen::Array3d& input, double dt);
  void resetState(void);

  /// @name Low-Pass Filter State Vector and Derivative
  //@{
  Eigen::Array3d filterState_ = Eigen::Array3d::Zero();
  Eigen::Array3d filterStateDer_ = Eigen::Array3d::Zero();
  //@}

private:
  /// @name Low-Pass Filter Gains
  //@{
  double gainP_ = 35530.5758439217;
  double gainQ_ = 266.572976289502;
  //@}
};

#endif  // UAV_LPF_H