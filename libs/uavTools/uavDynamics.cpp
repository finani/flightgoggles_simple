/**
 * @file uavDynamics.cpp
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Implementation of UAV Dynamics
 *
 */

#include "uavDynamics.h"

/**
 * @brief Construct a new UavDynamics::UavDynamics object
 *
 * @param nh ROS Nodehandle
 */
UavDynamics::UavDynamics(ros::NodeHandle nh)
  :  // Node handle
  node_(nh)
{
  //  Populate params
  if (!ros::param::get("/uav/flightgoggles_simple/ignore_collisions", ignoreCollisions_))
  {
    std::cout << "Did not get bool ignoreCollisions_ from the params, defaulting to false" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_simple/use_ratethrust_controller", useRateThrustController_))
  {
    std::cout << "Did not get bool useRateThrustController_ from the params, defaulting to true" << std::endl;
  }

  if (!ros::param::get("/uav/flightgoggles_simple/use_rungekutta4integrator", useRungeKutta4Integrator_))
  {
    std::cout << "Did not get bool useRungeKutta4Integrator_ from the params, defaulting to Explicit Euler integration"
              << std::endl;
  }

  if (!ros::param::get("/use_sim_time", useSimTime_))
  {
    std::cout << "Did not get bool useSimTime_ from the params, defaulting to false" << std::endl;
  }

  double vehicleMass;
  if (!ros::param::get("/uav/flightgoggles_simple/vehicle_mass", vehicleMass))
  {
    std::cout << "Did not get the vehicle mass from the params, defaulting to 1kg" << std::endl;
    vehicleMass = 1.;
  }

  double motorTimeconstant;
  if (!ros::param::get("/uav/flightgoggles_simple/motor_time_constant", motorTimeconstant))
  {
    std::cout << "Did not get the motor time constant from the params, defaulting to 0.02 s" << std::endl;
    motorTimeconstant = 0.02;
  }

  double motorRotationalInertia;
  if (!ros::param::get("/uav/flightgoggles_simple/motor_rotational_inertia", motorRotationalInertia))
  {
    std::cout << "Did not get the motor rotational inertia from the params, defaulting to 6.62e-6 kg m^2" << std::endl;
    motorRotationalInertia = 6.62e-6;
  }

  double momentArm;
  if (!ros::param::get("/uav/flightgoggles_simple/moment_arm", momentArm))
  {
    std::cout << "Did not get the moment arm from the params, defaulting to 0.08 m" << std::endl;
    momentArm = 0.08;
  }

  double thrustCoeff;
  if (!ros::param::get("/uav/flightgoggles_simple/thrust_coefficient", thrustCoeff))
  {
    std::cout << "Did not get the thrust coefficient from the params, defaulting to 1.91e-6 N/(rad/s)^2" << std::endl;
    thrustCoeff = 1.91e-6;
  }

  double torqueCoeff;
  if (!ros::param::get("/uav/flightgoggles_simple/torque_coefficient", torqueCoeff))
  {
    std::cout << "Did not get the torque coefficient from the params, defaulting to 2.6e-7 Nm/(rad/s)^2" << std::endl;
    torqueCoeff = 2.6e-7;
  }

  double dragCoeff;
  if (!ros::param::get("/uav/flightgoggles_simple/drag_coefficient", dragCoeff))
  {
    std::cout << "Did not get the drag coefficient from the params, defaulting to 0.1 N/(m/s)" << std::endl;
    dragCoeff = 0.1;
  }

  Eigen::Matrix3d aeroMomentCoefficient = Eigen::Matrix3d::Zero();
  if (!ros::param::get("/uav/flightgoggles_simple/aeromoment_coefficient_xx", aeroMomentCoefficient(0, 0)))
  {
    std::cout << "Did not get the aeromoment (x) from the params, defaulting to 0.003 Nm/(rad/s)^2" << std::endl;
    aeroMomentCoefficient(0, 0) = 0.003;
  }

  if (!ros::param::get("/uav/flightgoggles_simple/aeromoment_coefficient_yy", aeroMomentCoefficient(1, 1)))
  {
    std::cout << "Did not get the aeromoment (y) from the params, defaulting to 0.003 Nm/(rad/s)^2" << std::endl;
    aeroMomentCoefficient(1, 1) = 0.003;
  }

  if (!ros::param::get("/uav/flightgoggles_simple/aeromoment_coefficient_zz", aeroMomentCoefficient(2, 2)))
  {
    std::cout << "Did not get the aeromoment (z) from the params, defaulting to 0.003 Nm/(rad/s)^2" << std::endl;
    aeroMomentCoefficient(2, 2) = 0.003;
  }

  Eigen::Matrix3d vehicleInertia = Eigen::Matrix3d::Zero();
  if (!ros::param::get("/uav/flightgoggles_simple/vehicle_inertia_xx", vehicleInertia(0, 0)))
  {
    std::cout << "Did not get the inertia (x) from the params, defaulting to 0.0049 kg m^2" << std::endl;
    vehicleInertia(0, 0) = 0.0049;
  }

  if (!ros::param::get("/uav/flightgoggles_simple/vehicle_inertia_yy", vehicleInertia(1, 1)))
  {
    std::cout << "Did not get the inertia (y) from the params, defaulting to 0.0049 kg m^2" << std::endl;
    vehicleInertia(1, 1) = 0.0049;
  }

  if (!ros::param::get("/uav/flightgoggles_simple/vehicle_inertia_zz", vehicleInertia(2, 2)))
  {
    std::cout << "Did not get the inertia (z) from the params, defaulting to 0.0069 kg m^2" << std::endl;
    vehicleInertia(2, 2) = 0.0069;
  }

  double maxPropSpeed;
  if (!ros::param::get("/uav/flightgoggles_simple/max_prop_speed", maxPropSpeed))
  {
    std::cout << "Did not get the max prop speed from the params, defaulting to 2200 rad/s" << std::endl;
    maxPropSpeed = 2200.;
  }

  double momentProcessNoiseAutoCorrelation;
  if (!ros::param::get("/uav/flightgoggles_simple/moment_process_noise", momentProcessNoiseAutoCorrelation))
  {
    std::cout << "Did not get the moment process noise from the params, defaulting to 1.25e-7 (Nm)^2 s" << std::endl;
    momentProcessNoiseAutoCorrelation = 1.25e-7;
  }

  double forceProcessNoiseAutoCorrelation;
  if (!ros::param::get("/uav/flightgoggles_simple/force_process_noise", forceProcessNoiseAutoCorrelation))
  {
    std::cout << "Did not get the force process noise from the params, defaulting to 0.0005 N^2 s" << std::endl;
    forceProcessNoiseAutoCorrelation = 0.0005;
  }

  std::vector<double> initPose(7);
  if (!ros::param::get("/uav/flightgoggles_simple/init_pose", initPose))
  {
    // Start a few meters above the ground.
    std::cout << "Did NOT find initial pose from param file" << std::endl;

    initPose.at(0) = -10.5;
    initPose.at(1) = -18.5;
    initPose.at(2) = -1.5;
    initPose.at(6) = 1.0;
  }

  // Set gravity vector according to ROS reference axis system, see header file
  Eigen::Vector3d gravity(0., 0., -9.81);

  // Create quadcopter simulator
  multicopterSim_ =
      new MulticopterDynamicsSim(4, thrustCoeff, torqueCoeff, 0., maxPropSpeed, motorTimeconstant,
                                 motorRotationalInertia, vehicleMass, vehicleInertia, aeroMomentCoefficient, dragCoeff,
                                 momentProcessNoiseAutoCorrelation, forceProcessNoiseAutoCorrelation, gravity);

  // Set and publish motor transforms for the four motors
  Eigen::Isometry3d motorFrame = Eigen::Isometry3d::Identity();
  motorFrame.translation() = Eigen::Vector3d(momentArm, momentArm, 0.);
  multicopterSim_->setMotorFrame(motorFrame, -1, 0);

  publishStaticMotorTransform(ros::Time::now(), "uav/imu", "uav/motor0", motorFrame);

  motorFrame.translation() = Eigen::Vector3d(-momentArm, momentArm, 0.);
  multicopterSim_->setMotorFrame(motorFrame, 1, 1);

  publishStaticMotorTransform(ros::Time::now(), "uav/imu", "uav/motor1", motorFrame);

  motorFrame.translation() = Eigen::Vector3d(-momentArm, -momentArm, 0.);
  multicopterSim_->setMotorFrame(motorFrame, -1, 2);

  publishStaticMotorTransform(ros::Time::now(), "uav/imu", "uav/motor2", motorFrame);

  motorFrame.translation() = Eigen::Vector3d(momentArm, -momentArm, 0.);
  multicopterSim_->setMotorFrame(motorFrame, 1, 3);

  publishStaticMotorTransform(ros::Time::now(), "uav/imu", "uav/motor3", motorFrame);

  // Set initial conditions
  initPosition_ << initPose.at(0), initPose.at(1), initPose.at(2);
  initAttitude_.coeffs() << initPose.at(3), initPose.at(4), initPose.at(5), initPose.at(6);
  initAttitude_.normalize();
  initPropSpeed_ = sqrt(vehicleMass / 4. * 9.81 / thrustCoeff);

  multicopterSim_->setVehiclePosition(initPosition_, initAttitude_);
  multicopterSim_->setMotorSpeed(initPropSpeed_);

  // Get and set IMU parameters
  double accBiasProcessNoiseAutoCorrelation;
  if (!ros::param::get("/uav/flightgoggles_imu/accelerometer_biasprocess", accBiasProcessNoiseAutoCorrelation))
  {
    std::cout << "Did not get the accelerometer bias process auto corr from the params, defaulting to 1.0e-7 m^2/s^5"
              << std::endl;
    accBiasProcessNoiseAutoCorrelation = 1.0e-7;
  }

  double gyroBiasProcessNoiseAutoCorrelation;
  if (!ros::param::get("/uav/flightgoggles_imu/gyroscope_biasprocess", gyroBiasProcessNoiseAutoCorrelation))
  {
    std::cout << "Did not get the gyroscope bias process auto corr from the params, defaulting to 1.0e-7 rad^2/s^3"
              << std::endl;
    gyroBiasProcessNoiseAutoCorrelation = 1.0e-7;
  }

  if (!ros::param::get("/uav/flightgoggles_imu/accelerometer_biasinitvar", accBiasInitVar_))
  {
    std::cout << "Did not get the accelerometer bias initial value var from the params, defaulting to 0.005 (m/s^2)^2"
              << std::endl;
    accBiasInitVar_ = 0.005;
  }

  if (!ros::param::get("/uav/flightgoggles_imu/gyroscope_biasinitvar", gyroBiasInitVar_))
  {
    std::cout << "Did not get the gyroscope bias initial value var from the params, defaulting to 0.003 (rad/s)^2"
              << std::endl;
    gyroBiasInitVar_ = 0.003;
  }

  multicopterSim_->imu_.setBias(accBiasInitVar_, gyroBiasInitVar_, accBiasProcessNoiseAutoCorrelation,
                                gyroBiasProcessNoiseAutoCorrelation);

  if (!ros::param::get("/uav/flightgoggles_imu/accelerometer_variance", accMeasNoiseVariance_))
  {
    std::cout << "Did not get the accelerometer variance from the params, defaulting to 0.005 m^2/s^4" << std::endl;
    accMeasNoiseVariance_ = 0.005;
  }

  if (!ros::param::get("/uav/flightgoggles_imu/gyroscope_variance", gyroMeasNoiseVariance_))
  {
    std::cout << "Did not get the gyroscope variance from the params, defaulting to 0.003 rad^2/s^2" << std::endl;
    gyroMeasNoiseVariance_ = 0.003;
  }
  multicopterSim_->imu_.setNoiseVariance(accMeasNoiseVariance_, gyroMeasNoiseVariance_);

  // Only enable clock scaling when simtime is enabled.
  if (useSimTime_)
  {
    if (!ros::param::get("/uav/flightgoggles_simple/clockscale", clockScale))
    {
      std::cout << "Using sim_time and did not get a clock scaling value. Defaulting to automatic clock scaling."
                << std::endl;
      useAutomaticClockscale_ = true;
    }
  }

  // Print several parameters to terminal
  std::cout << "Ignore collisions: " << ignoreCollisions_ << std::endl;
  std::cout << "Initial position: " << initPosition_(0) << ", " << initPosition_(1) << ", " << initPosition_(2)
            << std::endl;
  std::cout << "Initial attitude: " << initAttitude_.coeffs()(0) << ", " << initAttitude_.coeffs()(1) << ", "
            << initAttitude_.coeffs()(2) << ", " << initAttitude_.coeffs()(3) << std::endl;

  // Init subscribers and publishers
  /*Allow for up to 100ms sim time buffer of outgoing IMU messages.
    This should improve IMU integration methods on slow client nodes (see issue #63). */
  imuPub_ = node_.advertise<sensor_msgs::Imu>("/uav/sensors/imu", 96);
  odomPub_ = node_.advertise<nav_msgs::Odometry>("/uav/odometry", 96);
  inputCommandSub_ = node_.subscribe("/uav/input/rateThrust", 1, &UavDynamics::inputCallback, this);
  inputMotorspeedCommandSub_ = node_.subscribe("/uav/input/motorspeed", 1, &UavDynamics::inputMotorspeedCallback, this);
  collisionSub_ = node_.subscribe("/uav/collision", 1, &UavDynamics::collisionCallback, this);
  armSub_ = node_.subscribe("/uav/input/arm", 1, &UavDynamics::armCallback, this);
  resetSub_ = node_.subscribe("/uav/input/reset", 1, &UavDynamics::resetCallback, this);
  frameRateSub_ = node_.subscribe("/uav/camera/debug/fps", 1, &UavDynamics::fpsCallback, this);

  if (useSimTime_)
  {
    clockPub_ = node_.advertise<rosgraph_msgs::Clock>("/clock", 1);
    clockPub_.publish(currentTime_);
  }
  else
  {
    // Get the current time if we are using wall time. Otherwise, use 0 as initial clock.
    currentTime_ = ros::Time::now();
  }

  // Init main simulation loop
  simulationLoopTimer_ =
      node_.createWallTimer(ros::WallDuration(dt_secs / clockScale), &UavDynamics::simulationLoopTimerCallback, this);
  simulationLoopTimer_.start();
}

/**
 * @brief Callback to handle the frame rate from unity
 * @param msg Float msg of frame rate in sim time from unity
 */
void UavDynamics::fpsCallback(std_msgs::Float32::Ptr msg)
{
  actualFps = msg->data;
}

/**
 * @brief Main Simulator loop
 * @param event Wall clock timer event
 */
void UavDynamics::simulationLoopTimerCallback(const ros::WallTimerEvent& event)
{
  // Step the time forward
  if (useSimTime_)
  {
    currentTime_ += ros::Duration(dt_secs);
    clockPub_.publish(currentTime_);
  }
  else
  {
    ros::Time loopStartTime = ros::Time::now();
    dt_secs = (loopStartTime - currentTime_).toSec();
    currentTime_ = loopStartTime;
  }

  // In case of collision reset state and disarm
  if (resetRequested_ || (hasCollided_ && !ignoreCollisions_))
  {
    lpf_.resetState();
    pid_.resetState();
    resetState();
    lastCommandMsg_.reset();
    lastMotorspeedCommandMsg_.reset();
    hasCollided_ = false;
    armed_ = false;
    resetRequested_ = false;
    return;
  }

  // Only propagate simulation if armed
  if (armed_)
  {
    Eigen::Array4d propSpeedCommand_ = Eigen::Array4d::Zero();

    if (useRateThrustController_)
    {
      // Proceed LPF state based on gyro measurement
      Eigen::Array3d imuGyroOutput = imuGyroOutput_.array();
      lpf_.proceedState(imuGyroOutput, dt_secs);

      // PID compute motor speed commands
      if (lastCommandMsg_)
      {
        Eigen::Array3d angular_rates(lastCommandMsg_->angular_rates.x, lastCommandMsg_->angular_rates.y,
                                     lastCommandMsg_->angular_rates.z);
        pid_.controlUpdate(angular_rates, lastCommandMsg_->thrust.z, lpf_.filterState_, lpf_.filterStateDer_,
                           propSpeedCommand_, dt_secs);
      }
    }
    else
    {
      if (lastMotorspeedCommandMsg_)
      {
        for (size_t motorIndx = 0; motorIndx < 4; motorIndx++)
        {
          propSpeedCommand_(motorIndx) = lastMotorspeedCommandMsg_->angular_velocities[motorIndx];
        }
      }
    }

    // Proceed quadcopter dynamics
    std::vector<double> propSpeedCommand(4, 0.);
    propSpeedCommand.at(0) = propSpeedCommand_(0);
    propSpeedCommand.at(1) = propSpeedCommand_(1);
    propSpeedCommand.at(2) = propSpeedCommand_(2);
    propSpeedCommand.at(3) = propSpeedCommand_(3);
    if (useRungeKutta4Integrator_)
    {
      multicopterSim_->proceedState_RK4(dt_secs, propSpeedCommand);
    }
    else
    {
      multicopterSim_->proceedState_ExplicitEuler(dt_secs, propSpeedCommand);
    }

    // Get IMU measurements
    multicopterSim_->getIMUMeasurement(imuAccOutput_, imuGyroOutput_);

    // Publish IMU measurements
    publishIMUMeasurement();
  }

  // Publish quadcopter state
  publishState();

  // Update clockscale if necessary
  if (actualFps != -1 && actualFps < 1e3 && useSimTime_ && useAutomaticClockscale_)
  {
    clockScale = (actualFps / 55.0);
    simulationLoopTimer_.stop();
    simulationLoopTimer_.setPeriod(ros::WallDuration(dt_secs / clockScale));
    simulationLoopTimer_.start();
  }
}

/**
 * @brief Handle incoming rate thrust message
 * @param msg rateThrust message from keyboard/ joystick controller
 */
void UavDynamics::inputCallback(mav_msgs::RateThrust::Ptr msg)
{
  lastCommandMsg_ = msg;
}

/**
 * @brief Handle arming message
 * @param msg Empty message, this will be recieved when drone is to be armed
 */
void UavDynamics::armCallback(std_msgs::Empty::Ptr msg)
{
  armed_ = true;
}

/**
 * @brief Handle reset message
 * @param msg Empty message, this will be recieved when drone is to be reset
 */
void UavDynamics::resetCallback(std_msgs::Empty::Ptr msg)
{
  resetRequested_ = true;
}

/**
 * @brief Handle incoming motor speed command message
 * @param msg Actuators message containing the motor speed commands
 */
void UavDynamics::inputMotorspeedCallback(mav_msgs::Actuators::Ptr msg)
{
  lastMotorspeedCommandMsg_ = msg;
}

/**
 * @brief Handle the checking of collisions
 * @param msg Empty message, this will be recieved when a collision is detected
 */
void UavDynamics::collisionCallback(std_msgs::Empty::Ptr msg)
{
  hasCollided_ = true;
}

/**
 * @brief Reset state to initial
 */
void UavDynamics::resetState(void)
{
  multicopterSim_->setVehiclePosition(initPosition_, initAttitude_);
  multicopterSim_->setMotorSpeed(initPropSpeed_);
  multicopterSim_->imu_.setBias(accBiasInitVar_, gyroBiasInitVar_);
}

/**
 * @brief Publish UAV state transform message
 */
void UavDynamics::publishState(void)
{
  geometry_msgs::TransformStamped transform;

  transform.header.stamp = currentTime_;
  transform.header.frame_id = "world";

  Eigen::Vector3d position = multicopterSim_->getVehiclePosition();
  Eigen::Quaterniond attitude = multicopterSim_->getVehicleAttitude();

  transform.transform.translation.x = position(0);
  transform.transform.translation.y = position(1);
  transform.transform.translation.z = position(2);

  transform.transform.rotation.x = attitude.x();
  transform.transform.rotation.y = attitude.y();
  transform.transform.rotation.z = attitude.z();
  transform.transform.rotation.w = attitude.w();

  transform.child_frame_id = "uav/imu";

  tfPub_.sendTransform(transform);

  nav_msgs::Odometry odometrymsg;

  odometrymsg.header.stamp = currentTime_;
  odometrymsg.header.frame_id = "world";
  odometrymsg.child_frame_id = "uav/imu";

  odometrymsg.pose.pose.position.x = position(0);
  odometrymsg.pose.pose.position.y = position(1);
  odometrymsg.pose.pose.position.z = position(2);

  odometrymsg.pose.pose.orientation.x = attitude.x();
  odometrymsg.pose.pose.orientation.y = attitude.y();
  odometrymsg.pose.pose.orientation.z = attitude.z();
  odometrymsg.pose.pose.orientation.w = attitude.w();

  odometrymsg.pose.covariance[0] = -1.;

  Eigen::Vector3d velocity = multicopterSim_->getVehicleVelocity();
  Eigen::Vector3d angularvelocity = multicopterSim_->getVehicleAngularVelocity();

  Eigen::Vector3d velocityBodyFrame = attitude.inverse() * velocity;

  odometrymsg.twist.twist.linear.x = velocityBodyFrame(0);
  odometrymsg.twist.twist.linear.y = velocityBodyFrame(1);
  odometrymsg.twist.twist.linear.z = velocityBodyFrame(2);

  odometrymsg.twist.twist.angular.x = angularvelocity(0);
  odometrymsg.twist.twist.angular.y = angularvelocity(1);
  odometrymsg.twist.twist.angular.z = angularvelocity(2);

  odometrymsg.twist.covariance[0] = -1.;

  odomPub_.publish(odometrymsg);
}

/**
 * @brief Publish IMU measurement message
 */
void UavDynamics::publishIMUMeasurement(void)
{
  sensor_msgs::Imu meas;

  meas.header.stamp = currentTime_;

  // Per message spec: set to -1 since orientation is not populated
  meas.orientation_covariance[0] = -1;

  meas.angular_velocity.x = imuGyroOutput_(0);
  meas.linear_acceleration.x = imuAccOutput_(0);

  meas.angular_velocity.y = imuGyroOutput_(1);
  meas.linear_acceleration.y = imuAccOutput_(1);

  meas.angular_velocity.z = imuGyroOutput_(2);
  meas.linear_acceleration.z = imuAccOutput_(2);

  meas.angular_velocity_covariance[0] = gyroMeasNoiseVariance_;
  meas.linear_acceleration_covariance[0] = accMeasNoiseVariance_;
  for (size_t i = 1; i < 8; i++)
  {
    if (i == 4)
    {
      meas.angular_velocity_covariance[i] = gyroMeasNoiseVariance_;
      meas.linear_acceleration_covariance[i] = accMeasNoiseVariance_;
    }
    else
    {
      meas.angular_velocity_covariance[i] = 0.;
      meas.linear_acceleration_covariance[i] = 0.;
    }
  }

  meas.angular_velocity_covariance[8] = gyroMeasNoiseVariance_;
  meas.linear_acceleration_covariance[8] = accMeasNoiseVariance_;

  imuPub_.publish(meas);
}

/**
 * @brief Publish static transform from UAV centroid to motor
 *
 * @param timeStamp Tf timestamp
 * @param frame_id Parent (UAV) frame ID
 * @param child_frame_id Child (motor) frame ID
 * @param motorFrame Transformation
 */
void UavDynamics::publishStaticMotorTransform(const ros::Time& timeStamp, const char* frame_id,
                                              const char* child_frame_id, const Eigen::Isometry3d& motorFrame)
{
  geometry_msgs::TransformStamped transformMotor;

  transformMotor.header.stamp = timeStamp;
  transformMotor.header.frame_id = frame_id;
  transformMotor.transform.translation.x = motorFrame.translation()(0);
  transformMotor.transform.translation.y = motorFrame.translation()(1);
  transformMotor.transform.translation.z = motorFrame.translation()(2);

  Eigen::Quaterniond motorAttitude(motorFrame.linear());

  transformMotor.transform.rotation.x = motorAttitude.x();
  transformMotor.transform.rotation.y = motorAttitude.y();
  transformMotor.transform.rotation.z = motorAttitude.z();
  transformMotor.transform.rotation.w = motorAttitude.w();
  transformMotor.child_frame_id = child_frame_id;

  staticTfPub_.sendTransform(transformMotor);
}
