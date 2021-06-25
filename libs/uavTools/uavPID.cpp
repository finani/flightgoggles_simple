/**
 * @file uavPID.cpp
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Implementation of UAV PID Controller
 *
 */

#include "uavPID.h"

/**
 * @brief Construct a new UavPID::UavPID object
 *
 */
UavPID::UavPID()
  : propGain_(9., 9., 9.)
  , intGain_(3., 3., 3.)
  , derGain_(0.3, 0.3, 0.3)
  , intState_(0., 0., 0.)
  , intBound_(1000., 1000., 1000.)
  , vehicleInertia_(0.0049, 0.0049, 0.0069)
{
  // Set parameters

  if (!ros::param::get("/uav/flightgoggles_pid/gain_p_roll", propGain_[0]))
  {
    std::cout << "Did not get the PID gain p roll from the params, defaulting to 9.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_i_roll", intGain_[0]))
  {
    std::cout << "Did not get the PID gain i roll from the params, defaulting to 3.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_d_roll", derGain_[0]))
  {
    std::cout << "Did not get the PID gain d roll from the params, defaulting to 0.3" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_p_pitch", propGain_[1]))
  {
    std::cout << "Did not get the PID gain p pitch from the params, defaulting to 9.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_i_pitch", intGain_[1]))
  {
    std::cout << "Did not get the PID gain i pitch from the params, defaulting to 3.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_d_pitch", derGain_[1]))
  {
    std::cout << "Did not get the PID gain d pitch from the params, defaulting to 0.3" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_p_yaw", propGain_[2]))
  {
    std::cout << "Did not get the PID gain p yaw from the params, defaulting to 9.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_i_yaw", intGain_[2]))
  {
    std::cout << "Did not get the PID gain i yaw from the params, defaulting to 3.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/gain_d_yaw", derGain_[2]))
  {
    std::cout << "Did not get the PID gain d yaw from the params, defaulting to 0.3" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/int_bound_roll", intBound_[0]))
  {
    std::cout << "Did not get the PID roll integrator bound from the params, defaulting to 1000.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/int_bound_pitch", intBound_[1]))
  {
    std::cout << "Did not get the PID pitch integrator bound from the params, defaulting to 1000.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/int_bound_yaw", intBound_[2]))
  {
    std::cout << "Did not get the PID yaw integrator bound from the params, defaulting to 1000.0" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/vehicle_inertia_xx", vehicleInertia_[0]))
  {
    std::cout << "Did not get the PID inertia (x) from the params, defaulting to 0.0049 kg m^2" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/vehicle_inertia_yy", vehicleInertia_[1]))
  {
    std::cout << "Did not get the PID inertia (y) from the params, defaulting to 0.0049 kg m^2" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/vehicle_inertia_zz", vehicleInertia_[2]))
  {
    std::cout << "Did not get the PID inertia (z) from the params, defaulting to 0.0069 kg m^2" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/moment_arm", momentArm_))
  {
    std::cout << "Did not get the PID moment arm from the params, defaulting to 0.08 m" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/thrust_coefficient", thrustCoeff_))
  {
    std::cout << "Did not get the PID thrust coefficient from the params, defaulting to 1.91e-6 N/(rad/s)^2"
              << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_pid/torque_coefficient", torqueCoeff_))
  {
    std::cout << "Did not get the PID torque coefficient from the params, defaulting to 2.6e-7 Nm/(rad/s)^2"
              << std::endl;
  }
}

/**
 * @brief Compute motor speed commands based on angular rate and thrust inputs
 *
 * @param command Angular rate command
 * @param thrustCommand Thrust command
 * @param curval Current vehicle angular rate
 * @param curder Current vehicle angular acceleration
 * @param propSpeedCommand Motor speed command vector output
 * @param dt Time step used for PID integrators
 */
void UavPID::controlUpdate(Eigen::Array3d& command, double thrustCommand, Eigen::Array3d& curval,
                           Eigen::Array3d& curder, Eigen::Array4d& propSpeedCommand, double dt)
{
  Eigen::Array3d stateDev = command - curval;
  intState_ += stateDev * dt;
  intState_ = intState_.max(-intBound_);
  intState_ = intState_.min(intBound_);
  Eigen::Array3d angAccCommand = propGain_ * stateDev + intGain_ * intState_ + derGain_ * -curder;

  thrustMixing(propSpeedCommand, angAccCommand, thrustCommand);
}

/**
 * @brief Compute motor speed commands based on angular acceleration and thrust commands
 *
 * @param propSpeedCommand Motor speed command vector output
 * @param angAccCommand Angular acceleration command
 * @param thrustCommand Thurst command
 */
void UavPID::thrustMixing(Eigen::Array4d& propSpeedCommand, Eigen::Array3d& angAccCommand, double thrustCommand)
{
  // Compute torque and thrust vector
  Eigen::Array4d momentThrust = Eigen::Array4d::Zero();
  momentThrust << vehicleInertia_ * angAccCommand, thrustCommand;

  // Compute signed, squared motor speed values
  Eigen::Matrix4d motorMixer = Eigen::Matrix4d::Zero();
  motorMixer.row(0) = Eigen::Vector4d(1., -1., -1., 1.);   // Motor 1
  motorMixer.row(1) = Eigen::Vector4d(1., 1., 1., 1.);     // Motor 2
  motorMixer.row(2) = Eigen::Vector4d(-1., 1., -1., 1.);   // Motor 3
  motorMixer.row(3) = Eigen::Vector4d(-1., -1., -1., 1.);  // Motor 4
  Eigen::Array4d motorCoeff = Eigen::Array4d::Zero();
  motorCoeff << 4 * momentArm_ * thrustCoeff_ * Eigen::Array3d::Ones(), 4 * thrustCoeff_;
  Eigen::Array4d motorSpeedsSquared = motorMixer * (momentThrust / motorCoeff).matrix();

  // Compute signed motor speed values
  propSpeedCommand = motorSpeedsSquared.sign() * sqrt(motorSpeedsSquared.sign() * motorSpeedsSquared);
}

/**
 * @brief Reset PID controller integrator state
 *
 */
void UavPID::resetState(void)
{
  intState_ = Eigen::Array3d::Zero();
}