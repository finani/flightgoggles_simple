/**
 * @file uavLPF.cpp
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Implementation of UAV Low Pass Filter
 *
 */

#include "uavLPF.h"

/**
 * @brief Construct a new Uav_LowPassFilter::Uav_LowPassFilter object
 *
 */
UavLPF::UavLPF() : filterState_(0., 0., 0.), filterStateDer_(0., 0., 0.)
{
  double temp1;
  double temp2;

  // Get filter gains
  if ((!ros::param::get("/uav/flightgoggles_lpf/gain_p", temp1)) ||
      (!ros::param::get("/uav/flightgoggles_lpf/gain_q", temp2)))
  {
    std::cout << "Did not get the LPF gain_p and/or gain_q from the params, defaulting to 30 Hz cutoff freq."
              << std::endl;
  }
  else
  {
    gainP_ = temp1;
    gainQ_ = temp2;
  }
}

/**
 * @brief Propagates the state using trapezoidal integration
 * @param input Filter input
 * @param dt Time step
 */
void UavLPF::proceedState(Eigen::Array3d& input, double dt)
{
  double det = gainP_ * dt * dt + gainQ_ * dt + 1.;
  Eigen::Array3d stateDer = Eigen::Array3d::Zero();
  stateDer = (filterStateDer_ + gainP_ * dt * input) / det - (dt * gainP_ * filterState_) / det;
  filterState_ = (dt * (filterStateDer_ + gainP_ * dt * input)) / det + ((dt * gainQ_ + 1.) * filterState_) / det;
  filterStateDer_ = stateDer;
}

/**
 * @brief Reset the LPF state to zeros
 */
void UavLPF::resetState(void)
{
  filterState_ = Eigen::Array3d::Zero();
  filterStateDer_ = Eigen::Array3d::Zero();
}