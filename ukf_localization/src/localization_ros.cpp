#include <localization_ros.h>
#include <fstream>
#include <sys/stat.h>
#include <iomanip> // std::setprecision()

LocalizationRos::LocalizationRos(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      filter_(1e-4, 2, 0),
      baseLinkFrame_("base_link"),
      odomFrame_("odom"),
      tfListener_(tfBuffer_)
{
  loadParams();
  filter_.setInitialCovariance(params_.initialCov);
  filter_.setProcessNoise(params_.processNoiseCov);
  filter_.setParam(params_.INITIALIZATION_NLS, params_.threshold);
  filter_.reset();

  try
  {
    //baseLinkFrame_ = ukf_localization::load_frame("robot");
    // todo: remove temp output frame
    //    odomFrame_ = au_core::load_frame("odom");
    odomFrame_ = "temp_odom";
    odomPub_ = nh_.advertise<nav_msgs::Odometry>(
        "/Locater/Odom", 20);
    statePub_ = nh_.advertise<auv_nav_msg::State>(
        "/State", 20);
    imuSub_ = nh_.subscribe(
        "/Sensor/AHRS", 5, &LocalizationRos::imuCallback, this);

    dvlSub_ = nh_.subscribe(
        "/Sensor/DVL", 5, &LocalizationRos::dvlCallback, this);

    depthSub_ = nh_.subscribe<sensor_msgs::FluidPressure>(
        "/Sensor/Pressure", 5, &LocalizationRos::depthCallback, this);

    usbllongSub_ = nh_.subscribe(
        "/Sensor/USBLLONG", 5, &LocalizationRos::usbllongCallback, this);

    usblanglesSub_ = nh_.subscribe(
        "/Sensor/USBLANGLES", 5, &LocalizationRos::usblanglesCallback, this);

    recvimSub_ = nh_.subscribe(
        "/Sensor/RECVIM", 5, &LocalizationRos::usblrecvimCallback, this);

#ifdef BAG
    clockSub_ = nh_.subscribe<rosgraph_msgs::Clock>(
        "/clock", 5, &LocalizationRos::clockCallback, this);
#endif
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Unable to load topic/frame. Error: %s", e.what());
  }

  updateTimer_ = nh_.createTimer(ros::Duration(1. / params_.frequency),
                                 &LocalizationRos::update, this);
}

void LocalizationRos::reset()
{
  std::cout << "LocalizationRos::reset()" << std::endl;
  //time_record = 0;
  clearMeasurementQueue();

  tfBuffer_.clear();

  // clear all waiting callbacks
  ros::getGlobalCallbackQueue()->clear();
}

void LocalizationRos::update(const ros::TimerEvent &event)
{ // include update with measurment and predict
  // warn user if update loop takes too long
  // timefile<<"Start ";
  // double startupdate = ros::Time::now().toSec();
  const double last_cycle_duration =
      (event.current_real - event.last_expected).toSec();
  if (last_cycle_duration > 2 / params_.frequency)
  {
    ROS_WARN_STREAM("Failed to meet update rate and even more than twice of expected duration ! Last cycle took "
                    << std::setprecision(20) << last_cycle_duration << "; last duration is " << event.profile.last_duration << ". However, the frequency is " << params_.frequency << " so expected duration is less than" << 1. / params_.frequency);
  }
#ifdef BAG
  double currentTime = time_record;
#else
  double currentTime = ros::Time::now().toSec();
#endif

  if (!measurementQueue_.empty())
  {
    count_no_sensor = 0;
    int count_sensor = 0;
    //ROS_INFO("measurement is not empty");
    while (!measurementQueue_.empty() && ros::ok())
    {
      MeasurementPtr z = measurementQueue_.top();
      count_sensor++;

      // if measurement's time is later than now, wait until next iteration
      if (z->time > currentTime)
      {
        std::cout << "wrong time time of measurement > currentTime" << std::endl
                  << "And measurement time - currentTime is" << z->time - currentTime << std::endl;
        std::cout << "ros::Time::now() is " << ros::Time::now().toSec() << std::endl;
        break;
      }
      double deltaT = currentTime - z->time;
      if (deltaT > 10.0)
      {
        ROS_WARN(
            "In this updata cycle there is  measurement received but delta was very large. Suspect playing from bag file."
            "currenttime is %f; measurement time is %f",
            currentTime, filter_.getLastMeasurementTime());
        //deltaT = 0.01;
      }
      measurementQueue_.pop();
      // predict + update loop with measurement
      double start = ros::Time::now().toSec();

      filter_.processMeasurement(*(z.get()));
      // timefile<<"duration for processMeasurement is "<<  ros::Time::now().toSec() - start <<std::endl;
    }
    logfile << "In the update, there are " << count_sensor << " sensor measurements" << std::endl;
  }
  else if (filter_.isInitialized())
  { // only predict if initialized
    // no measurement call filter predict
    count_no_sensor++;
    // timefile<<" 11 ";
    double deltaT = currentTime - filter_.getLastMeasurementTime();
    if (deltaT > 10.0)
    {
      ROS_WARN(
          "In this updata cycle there is no measurement received. Delta was very large. Suspect playing from bag file."
          "currenttime is %f; last time is %f",
          currentTime, filter_.getLastMeasurementTime());
      //deltaT = 0.01;
    }

    if (count_no_sensor <= 5)
    {
      //timefile<<deltaT<<std::endl;
      double start = ros::Time::now().toSec();
      filter_.predict(deltaT);

      double duration = ros::Time::now().toSec() - start;
// logfile<<"duration for predict is "<<duration<<std::endl;
#ifdef BAG
      filter_.setLastMeasurementTime(time_record);
#else
      filter_.setLastMeasurementTime(currentTime);
#endif
      std::cout << "count_no_sensor is " << count_no_sensor << std::endl;
      ROS_WARN_THROTTLE(1.0, "No measurements recieved. Using prediction only.");
    }
  }

  statePub_.publish(getStateMessage());

  // publish message and frame transform
  if (filter_.isInitialized())
  {
    publishOdomTf();
 
  }
  // timefile << std::fixed <<std::setprecision(3)<< event.current_expected.toSec()<<"  "<< event.current_real.toSec()<< "  "<< event.last_expected.toSec()<<"  "<<  event.last_real.toSec() <<"  "<< startupdate <<"  " <<ros::Time::now().toSec()<<"  "<< ros::Time::now().toSec()- startupdate<<std::endl<<std::endl;
}

nav_msgs::Odometry LocalizationRos::getFilteredOdomMessage()
{
  // should only be called if filter is initialized
  assert(filter_.isInitialized());

  const Eigen::VectorXd &state = filter_.getState();
  const Eigen::MatrixXd &cov = filter_.getCovariance();

  Eigen::Quaterniond quat;
  quat = Eigen::AngleAxisd(state(StateYaw), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(state(StatePitch), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(state(StateRoll), Eigen::Vector3d::UnitX());

  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = state(StateX);
  odom.pose.pose.position.y = state(StateY);
  odom.pose.pose.position.z = state(StateZ);
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.pose.pose.orientation.w = quat.w();
  odom.twist.twist.linear.x = state(StateVx);
  odom.twist.twist.linear.y = state(StateVy);
  odom.twist.twist.linear.z = state(StateVz);
  odom.twist.twist.angular.x = state(StateVroll);
  odom.twist.twist.angular.y = state(StateVpitch);
  odom.twist.twist.angular.z = state(StateVyaw);
  // std::string dir = "/home/john/Desktop/auv_prj/Integrated_navigation/src/ukf_localization/sim_data";

  // std::string file = dir+"/log.txt";
  // std::ofstream logfile;
  // logfile.open(file.c_str(),std::ios::app);

  // logfile<<"odom angular x: "<<odom.pose.pose.orientation.x<<";y:"<<odom.pose.pose.orientation.y<<"; z: "<<odom.pose.pose.orientation.z<<"vx:"<<state(StateVx)<<";vy:"<<state(StateVy)<<";vz:"<<state(StateVz)<<std::endl;

  //ROS_INFO("odom angular x: %f, y: %f, z: %f,w:%f roll:%f, pitch:%f, yaw:%f,vx:%f, vy:%f,vz:%f",odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w,state(StateRoll), state(StatePitch), state(StateYaw),state(StateVx),state(StateVy),state(StateVz) );
  for (size_t i = 0; i < 6; ++i)
  {
    for (size_t j = 0; j < 6; ++j)
    {
      odom.pose.covariance[6 * i + j] = cov(i, j);
      odom.twist.covariance[6 * i + j] = cov(i + StateVx, j + StateVx);
    }
  }

  odom.header.stamp = ros::Time(filter_.getLastMeasurementTime());
  odom.header.frame_id = odomFrame_;
  odom.child_frame_id = "ahrs";
  return odom;
}
auv_nav_msg::State LocalizationRos::getStateMessage()
{
  Eigen::Matrix<double, 1, STATE_SIZE> stateVector = filter_.getState().transpose();
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> covarianceMatrix = filter_.getCovariance();
  statefile << std::fixed << std::setprecision(5) << ros::Time::now().toSec() - startSec << " " << stateVector << std::endl;
  auv_nav_msg::State state;
  state.position.x = stateVector(StateX);
  state.position.y = stateVector(StateY);
  state.position.z = stateVector(StateZ);
  state.rpy.x = stateVector(StateRoll);
  state.rpy.y = stateVector(StatePitch);
  state.rpy.z = stateVector(StateYaw);
  state.velocity.x = stateVector(StateVx);
  state.velocity.y = stateVector(StateVy);
  state.velocity.z = stateVector(StateVz);
  state.acceleration.x = stateVector(StateAx);
  state.acceleration.y = stateVector(StateAy);
  state.acceleration.z = stateVector(StateAz);
  state.angular_velocity.x = stateVector(StateVroll);
  state.angular_velocity.y = stateVector(StateVpitch);
  state.angular_velocity.z = stateVector(StateVyaw);
  state.beacon_pos.x = stateVector(StateBeaconX);
  state.beacon_pos.y = stateVector(StateBeaconY);
  state.beacon_pos.z = stateVector(StateBeaconZ);
  state.usbl_rpy.x = stateVector(StateUsblR);
  state.usbl_rpy.y = stateVector(StateUsblP);
  state.usbl_rpy.z = stateVector(StateUsblY);
  state.position_covariance_diag.x = covarianceMatrix(StateX, StateX);
  state.position_covariance_diag.y = covarianceMatrix(StateY, StateY);
  state.position_covariance_diag.z = covarianceMatrix(StateZ, StateZ);
  state.rpy_covariance_diag.x = covarianceMatrix(StateRoll, StateRoll);
  state.rpy_covariance_diag.y = covarianceMatrix(StatePitch, StatePitch);
  state.rpy_covariance_diag.z = covarianceMatrix(StateYaw, StateYaw);
  state.velocity_covariance_diag.x = covarianceMatrix(StateVx, StateVx);
  state.velocity_covariance_diag.y = covarianceMatrix(StateVy, StateVy);
  state.velocity_covariance_diag.z = covarianceMatrix(StateVz, StateVz);
  state.acceleration_covariance_diag.x = covarianceMatrix(StateAx, StateAx);
  state.acceleration_covariance_diag.y = covarianceMatrix(StateAy, StateAy);
  state.acceleration_covariance_diag.z = covarianceMatrix(StateAz, StateAz);
  state.angular_velocity_covariance_diag.x = covarianceMatrix(StateVroll, StateVroll);
  state.angular_velocity_covariance_diag.y = covarianceMatrix(StateVpitch, StateVpitch);
  state.angular_velocity_covariance_diag.z = covarianceMatrix(StateVyaw, StateVyaw);
  state.beacon_pos_covariance_diag.x = covarianceMatrix(StateBeaconX, StateBeaconX);
  state.beacon_pos_covariance_diag.y = covarianceMatrix(StateBeaconY, StateBeaconY);
  state.beacon_pos_covariance_diag.z = covarianceMatrix(StateBeaconZ, StateBeaconZ);
  state.usbl_rpy_covariance.x = covarianceMatrix(StateUsblR, StateUsblR);
  state.usbl_rpy_covariance.y = covarianceMatrix(StateUsblP, StateUsblP);
  state.usbl_rpy_covariance.z = covarianceMatrix(StateUsblY, StateUsblY);

  state.header.stamp = ros::Time::now();
  return state;
}
void LocalizationRos::publishOdomTf()
{
  // publish odom message
  // odomfile << std::fixed << std::setprecision(3) <<filteredState.header.stamp.toSec()-startSec<<" "<< filteredState.pose.pose.position.x<<" "<<filteredState.pose.pose.position.y<<" "<<filteredState.pose.pose.position.z<<" "<<filteredState.twist.twist.linear.x<<" "<<filteredState.twist.twist.linear.y<<" "<<filteredState.twist.twist.linear.z<<std::endl;
  //ROS_INFO("odom published \n stamp: %f", odomTransMsg.header.stamp.toSec());
  //ROS_INFO("[ x: %f y: %f z: %f]", odomTransMsg.transform.translation.x,odomTransMsg.transform.translation.y,odomTransMsg.transform.translation.z );
  nav_msgs::Odometry filteredState = getFilteredOdomMessage();
  double speed = sqrt(filteredState.twist.twist.linear.x * filteredState.twist.twist.linear.x + filteredState.twist.twist.linear.y * filteredState.twist.twist.linear.y + filteredState.twist.twist.linear.z * filteredState.twist.twist.linear.z);
  if (speed > 3 && (!Diverge))
  {
    Diverge = true;
    diverge_start = ros::Time::now().toSec();
    dist = 0;
  }
  if (Diverge)
  {
    dist = dist + speed * 1 / params_.frequency;
    std::cout << " diverge starts, distance is " << dist << ";time is " << ros::Time::now().toSec() - diverge_start << "; now err flag is " << ERR << std::endl;
    if (ros::Time::now().toSec() - diverge_start > 30)
    {
      if (dist > 100)
      {
        ERR = true;
      }
      else
      {
        Diverge = false;
        ERR = false;
        dist = 0;
        diverge_start = 0;
      }
    }
  }
  if (!ERR)
  {
    odomPub_.publish(filteredState);
  }

#ifndef BAG
  // broadcast odom frame
  geometry_msgs::TransformStamped w2imu;
  w2imu.header.stamp = filteredState.header.stamp;
  // std::cout<<"filteredState.header.frame_id:"<<filteredState.header.frame_id << std::endl;
  w2imu.header.frame_id = filteredState.header.frame_id;
  w2imu.child_frame_id = filteredState.child_frame_id;
  w2imu.transform.translation.x = filteredState.pose.pose.position.x;
  w2imu.transform.translation.y = filteredState.pose.pose.position.y;
  w2imu.transform.translation.z = filteredState.pose.pose.position.z;
  w2imu.transform.rotation = filteredState.pose.pose.orientation;
  odomBroadcaster_.sendTransform(w2imu);

  geometry_msgs::TransformStamped base2imu;
  base2imu.header.stamp = filteredState.header.stamp;
  base2imu.header.frame_id = "ahrs";
  base2imu.child_frame_id = baseLinkFrame_;
  base2imu.transform.translation.x = -0.4375;
  base2imu.transform.translation.y = 0;
  base2imu.transform.translation.z = 0.1876;
  base2imu.transform.rotation.w = 1;
  base2imu.transform.rotation.x = 0;
  base2imu.transform.rotation.y = 0;
  base2imu.transform.rotation.z = 0;
  odomBroadcaster_.sendTransform(base2imu);
#endif  
}

#ifdef BAG

void LocalizationRos::clockCallback(const rosgraph_msgs::ClockConstPtr &msg)
{
  time_record = msg->clock.toSec();
  std::cout << "ros::Time::now() is " << ros::Time::now().toSec() << std::endl;
  std::cout << " time record is " << time_record << std::endl;
  //std::cout<<"LocalizationRos::clockCallback: time record is "<<time_record<<std::endl;
}
#endif

void LocalizationRos::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  //std::cout<<"received"<<std::endl;
  // double imu_time = msg->header.stamp.toSec();
  double imu_time = ros::Time::now().toSec();
  if (imu_time - last_imu_time >= 0.04)
  {
    last_imu_time = imu_time;
    if (msg->orientation_covariance[0] > 0)
    {
      MeasurementPtr z = std::make_shared<Measurement>(IMU_SIZE);
      z->type = MeasurementTypeImu;
      z->time = ros::Time::now().toSec(); //msg->header.stamp.toSec();
      // time_record = z->time;
      z->covariance.setZero();
      // tf2::Transform targetFrameTrans = getTransformFrame(msg->header);
      tf2::Transform targetFrameTrans;
      targetFrameTrans.setIdentity();
      // orientation
      // note: IMU should be mounted such that RPY is in NED coord frame
      tf2::Quaternion q;
      tf2::fromMsg(msg->orientation, q);
#ifdef ENU
      tf2::Quaternion ned2enu;
      ned2enu.setRPY(M_PI, 0, M_PI / 2);
      q = ned2enu * q;
#else
      q = q.inverse();
#endif
      tf2::Matrix3x3 orientation(q);

      double roll, pitch, yaw;
      Eigen::Matrix3d matrix;
      // matrix<<orientation.getRow(0).getX(),orientation.getRow(0).getY(),orientation.getRow(0).getZ(),
      //     orientation.getRow(1).getX(),orientation.getRow(1).getY(),orientation.getRow(1).getZ(),
      //     orientation.getRow(2).getX(),orientation.getRow(2).getY(),orientation.getRow(2).getZ();

      // rotfile <<std::fixed<<std::setprecision(5)<<z->time<<" "<<std::endl<< matrix <<std::endl<<std::endl;
      orientation.getRPY(roll, pitch, yaw);

      // statefile << std::fixed<<std::setprecision(2)<<z->time<<" "<<roll<<" "<<pitch<<" "<<yaw<<std::endl;

      //orientation.getEulerYPR(yaw, pitch, roll);

      // roll = EigenQ.toRotationMatrix().eulerAngles(0,1,2)(0);
      // pitch = EigenQ.toRotationMatrix().eulerAngles(0,1,2)(1);
      // yaw = EigenQ.toRotationMatrix().eulerAngles(0,1,2)(2);
      // q = Eigen::AngleAxisf(data.euler_angle(2), Eigen::Vector3f::UnitZ())
      //     * Eigen::AngleAxisf(data.euler_angle(1), Eigen::Vector3f::UnitY())
      //     * Eigen::AngleAxisf(data.euler_angle(0), Eigen::Vector3f::UnitX());

      //Eigen::Quaterniond EigenQ(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
      //roll = EigenQ.toRotationMatrix().eulerAngles(2,1,0)(2);
      //pitch = EigenQ.toRotationMatrix().eulerAngles(2,1,0)(1);
      //yaw = EigenQ.toRotationMatrix().eulerAngles(2,1,0)(0);

      z->measurement(ImuRoll) = roll;
      z->measurement(ImuPitch) = pitch;
      z->measurement(ImuYaw) = yaw;

      z->covariance.block<3, 3>(ImuRoll, ImuRoll) =
          Eigen::Vector3d(msg->orientation_covariance[0],
                          msg->orientation_covariance[4],
                          msg->orientation_covariance[8])
              .asDiagonal();
      // logfile<< "orientation cov:"<<std::endl<<
      Eigen::Vector3d(msg->orientation_covariance[0],
                      msg->orientation_covariance[4],
                      msg->orientation_covariance[8]);
      // z->covariance.block<3, 3>(ImuRoll, ImuRoll) =
      //     Eigen::Vector3d(0,
      //                     0,
      //                     0)
      //         .asDiagonal();

      // angular velocity
      tf2::Vector3 angularVelocity(msg->angular_velocity.x, msg->angular_velocity.y,
                                   msg->angular_velocity.z);
      angularVelocity = targetFrameTrans.getBasis() * angularVelocity;
      z->measurement(ImuVroll) = angularVelocity.x();
      z->measurement(ImuVpitch) = angularVelocity.y();
      z->measurement(ImuVyaw) = angularVelocity.z();
      // rotate covariance matrix to base_link
      Eigen::MatrixXd covarianceRotated(3, 3);
      rotateCovariance(&(msg->angular_velocity_covariance[0]),
                       targetFrameTrans.getRotation(), covarianceRotated);
      z->covariance.block<3, 3>(ImuVroll, ImuVroll) = covarianceRotated;
#ifdef ENU_ACC
      tf2::Vector3 linearAcceleration(msg->linear_acceleration.y,
                                      msg->linear_acceleration.x,
                                      -msg->linear_acceleration.z);
#else
      // linear acceleration
      tf2::Vector3 linearAcceleration(msg->linear_acceleration.x,
                                      msg->linear_acceleration.y,
                                      msg->linear_acceleration.z);
      tf2::Vector3 gravity(0, 0, -9.81);

      linearAcceleration = linearAcceleration - orientation.inverse() * gravity;
#endif
      // note: we assume that if the sensor is placed at some non-zero offset from
      // the vehicle's center, the vehicle turns with constant velocity. This is
      // because we do not have angular acceleration
      linearAcceleration = targetFrameTrans.getBasis() * linearAcceleration;
      z->measurement(ImuAx) = linearAcceleration.x();
      z->measurement(ImuAy) = linearAcceleration.y();
      z->measurement(ImuAz) = linearAcceleration.z();
      //logfile<<"covariance for imu callback:"<<std::endl;
      Eigen::MatrixX3d covarianceMatrix = Eigen::Vector3d(msg->orientation_covariance[0],
                                                          msg->orientation_covariance[4], msg->orientation_covariance[8])
                                              .asDiagonal();
      //logfile<<covarianceMatrix<<std::endl;
      // rotate covariance matrix to base_link
      rotateCovariance(&(msg->linear_acceleration_covariance[0]),
                       targetFrameTrans.getRotation(), covarianceRotated);
      z->covariance.block<3, 3>(ImuAx, ImuAx) = covarianceRotated;

      // measure_log<<"imu"<<std::endl;
      // std::cout<<"2021 0801 received imu"<<std::endl;
      measurementQueue_.push(z);
    }
  }
}

void LocalizationRos::dvlCallback(const auv_nav_msg::DVLConstPtr &msg)
{
  MeasurementPtr z = std::make_shared<Measurement>(DVL_SIZE);
  z->type = MeasurementTypeDvl;
  // #ifdef BAG
  z->time = ros::Time::now().toSec(); //msg->header.stamp.toSec();

  //std::cout<< "#ifdef BAG/dvlCallback: time_record is "<<time_record<<std::endl;

  // #else
  // z->time = ros::Time::now().toSec();
  //std::cout<< "#else /dvlCallback: time_record is "<<z->time<<std::endl;

  // #endif
  // tf2::Transform targetFrameTrans = getTransformFrame(msg->header);
  // tf2::Transform targetFrameTrans;
  // targetFrameTrans.setIdentity();
  tf2::Quaternion q;
  tf2::Matrix3x3 R;
#ifdef ROTATE
  R.setValue(0.717219, 0.696831, -0.004789,
             -0.696764, 0.717223, 0.010524,
             0.010768, -0.004211, 0.999933);

  R.getRotation(q);
  tf2::Transform targetFrameTrans(q, tf2::Vector3(0.0, 0.0, 0.0));
#else
  tf2::Transform targetFrameTrans;
  targetFrameTrans.setIdentity();
#endif
  // std::cout <<"rotation:"<<targetFrameTrans.getRotation().x()<<","<<targetFrameTrans.getRotation().y()<<","<<targetFrameTrans.getRotation().z()<<","<<targetFrameTrans.getRotation().w()<<std::endl;
  if (msg->velocity_body.x > -99990 && sqrt(pow(msg->velocity_body.x, 2) + pow(msg->velocity_body.y, 2) + pow(msg->velocity_body.z, 2)) < 4000)
  {
    tf2::Vector3 linVel(msg->velocity_body.x, msg->velocity_body.y, msg->velocity_body.z);

    linVel = targetFrameTrans.getBasis() * linVel;
    // account for linear velocity as a result of sensor offset and
    // rotational velocity
    const Eigen::VectorXd &state = filter_.getState();
    tf2::Vector3 angVel(state(StateVroll), state(StateVpitch), state(StateVyaw));
    linVel += targetFrameTrans.getOrigin().cross(angVel);
    z->measurement(0) = linVel.x() / 1000;
    z->measurement(1) = linVel.y() / 1000;
    z->measurement(2) = linVel.z() / 1000;
    // rotate covariance matrix to base_link
    Eigen::MatrixXd covarianceRotated(3, 3);

    double error = params_.dvl_noise_sigma;
    double covariance[9] = {error * error, 0, 0, 0, error * error, 0, 0, 0, error * error};
    // rotateCovariance(&(msg->velocity_covariance[0]),
    //                  targetFrameTrans.getRotation(), covarianceRotated);
    rotateCovariance(&(covariance[0]),
                     targetFrameTrans.getRotation(), covarianceRotated);
    z->covariance.block<3, 3>(0, 0) = covarianceRotated;

    // measure_log<<"dvl"<<std::endl;
    // dvlfile << std::fixed << std::setprecision(3) << "received velocity :" << " "<<" time :"<<z->time <<" " << z->measurement(0)<<" "<<z->measurement(1) <<   " "<<z->measurement(2)<<std::endl;

    //std::cout<<"dvl time is "<<z->time<<std::endl;
    measurementQueue_.push(z);
  }
}

void LocalizationRos::depthCallback(const sensor_msgs::FluidPressureConstPtr &msg)
{
  MeasurementPtr z = std::make_shared<Measurement>(DEPTH_SIZE);
  z->type = MeasurementTypeDepth;
  z->time = ros::Time::now().toSec(); //msg->header.stamp.toSec(); //
  // tf2::Transform targetFrameTrans = getTransformFrame(msg->header);
  tf2::Transform targetFrameTrans;
  targetFrameTrans.setIdentity();
  // take into account positional offset of depth sensor
  z->measurement(0) = msg->fluid_pressure - targetFrameTrans.getOrigin().z();
  // z->covariance(0, 0) = msg->variance;
  z->covariance(0, 0) = params_.depth_noise_sigma * params_.depth_noise_sigma;

  //std::cout<<"depth time is "<<z->time<<std::endl;
  // std::cout << "depth received" << std::endl;
  measurementQueue_.push(z);
}

void LocalizationRos::usbllongCallback(const auv_nav_msg::USBLLONGConstPtr &msg)
{
  if (msg->accuracy < 20)
  {
    MeasurementPtr z = std::make_shared<Measurement>(ANGLE_SIZE);
    z->type = MeasurementTypeAngle;
    // z->time = ros::Time::now().toSec() - (msg->current_time - msg->measurement_time);//msg->header.stamp.toSec() - (msg->current_time - msg->measurement_time);
    z->time = ros::Time::now().toSec();
    double dist = sqrt(msg->pos_xyz.x * msg->pos_xyz.x + msg->pos_xyz.y * msg->pos_xyz.y + msg->pos_xyz.z * msg->pos_xyz.z);
    double bearing = atan2(msg->pos_xyz.y, msg->pos_xyz.x);
    double elevation = asin(msg->pos_xyz.z / dist);
    z->measurement << bearing, elevation;
    double error = asin(msg->accuracy / dist);
    std::cout << "In USBLLONG, the error is " << error << std::endl;
    logfile << "USBLLONG, bearing is " << bearing << "; elevation is " << elevation << std::endl;
    z->covariance << error * error, 0,
        0, error * error;
    const Eigen::VectorXd &state = filter_.getState();
    bool InLier = true;


    Eigen::Vector3d pos_enu;
    pos_enu << msg->pos_enu.x, msg->pos_enu.y, msg->pos_enu.z;
    if (params_.PRIOR)
    {
      InLier = (state(StateY) + pos_enu.x() > -47) && (state(StateY) + pos_enu.x() < -29) 
                && (state(StateX) + pos_enu.y() > -40) && (state(StateX) + pos_enu.y() < -23)
                && (-state(StateZ) + pos_enu.z() > -12) && (-state(StateZ) + pos_enu.z() < 5);
    }
    else {
      InLier = (error < 0.3 );
    }
    Eigen::Vector3d beacon_pos;
    beacon_pos << state(StateY) + pos_enu.x(), state(StateX) + pos_enu.y(), -state(StateZ) + pos_enu.z();
    std::cout << "beacon_pos is " << beacon_pos.transpose() << std::endl;
    if (InLier)//&& state(StateZ) > 4)
    {
      
      measurementQueue_.push(z);
      std::cout << "Accept the USBLLONG (PRIOR: " << params_.PRIOR << std::endl;
    }
    else
    {
      std::cout << "Reject the USBLLONG" << std::endl;
    }
  }
}
void LocalizationRos::usblanglesCallback(const auv_nav_msg::USBLANGLESConstPtr &msg)
{
  logfile << "received angle; now time in second is " << ros::Time::now().toSec() - startSec << std::endl;

  std::cout << "received angle; now time in second is " << ros::Time::now().toSec() - startSec << std::endl;
  if (msg->accuracy > 0 && msg->accuracy < 0.15)
  {
    logfile << "accepted angle" << std::endl;

    MeasurementPtr z = std::make_shared<Measurement>(ANGLE_SIZE);
    z->type = MeasurementTypeAngle;
    // z->time = ros::Time::now().toSec() - (msg->current_time - msg->measurement_time);//msg->header.stamp.toSec() - (msg->current_time - msg->measurement_time);
    z->time = ros::Time::now().toSec();
    z->measurement << msg->lbearing, msg->lelevation;
    logfile << "USBLANGLES, bearing is " << msg->lbearing << "; elevation is " << msg->lelevation << std::endl;

    double error;

    error = msg->accuracy;
    std::cout << "now time in second is " << ros::Time::now().toSec() - startSec << std::endl;
    z->covariance << error * error, 0,
        0, error * error;
    measurementQueue_.push(z);
  }
  else
  {
    logfile << "USBLANGLES rejected, of which the accuracy is " << msg->accuracy << std::endl;

    std::cout << "USBLANGLES rejected, of which the accuracy is " << msg->accuracy << std::endl;
  }
}

void LocalizationRos::usblrecvimCallback(const auv_nav_msg::RECVIMConstPtr &msg)
{
  MeasurementPtr z = std::make_shared<Measurement>(RECVIM_SIZE);
  z->type = MeasurementTypeRecvim;
  z->time = ros::Time::now().toSec(); //msg->header.stamp.toSec();
  std::string str = msg->data;
  double depth = 0;
  double depth_sigma = 1.0e5;
  if (index_first(str, "DEPTH") != -1)
  {
    auto split_ret = split(str, ',');
    depth = std::stof(split_ret[1]);
    depth_sigma = 0.6;
  }
  // depth = 10;
  // depth_sigma = 0.6;
  z->measurement << msg->velocity, depth;
  double speed_error = 0.3; //0.03;
  speed_error = params_.recvim_noise_sigma;

  z->covariance << speed_error * speed_error, 0,
      0, depth_sigma * depth_sigma;
  measurementQueue_.push(z);
}

void LocalizationRos::loadParams()
{
  private_nh_.param("frequency", params_.frequency, 10.0);
  private_nh_.param("dvl_noise_sigma", params_.dvl_noise_sigma, 1e-2);
  private_nh_.param("depth_noise_sigma", params_.depth_noise_sigma, 0.5);
  private_nh_.param("recvim_noise_sigma", params_.recvim_noise_sigma, 0.03);
  private_nh_.param("threshold", params_.threshold, 5.0e-3);
  private_nh_.getParam("PRIOR", params_.PRIOR);
  private_nh_.getParam("INITIALIZATION_NLS", params_.INITIALIZATION_NLS);
  loadMatrixFromParams(params_.initialCov, "initial_estimate_covariance");
  loadMatrixFromParams(params_.processNoiseCov, "process_noise_covariance");
  std::cout << "loaded threshold: " << params_.threshold << std::endl;
  std::cout << "loaded INITIALIZATION_NLS: " << params_.INITIALIZATION_NLS << std::endl;
  std::cout << "loaded prior: " << params_.PRIOR << std::endl;
  logfile << "loaded noise:" << std::endl
          << params_.processNoiseCov << std::endl;
}

void LocalizationRos::loadMatrixFromParams(Eigen::MatrixXd &mat,
                                           const std::string &key)
{
  size_t size = mat.rows();
  mat.setZero();
  XmlRpc::XmlRpcValue param;

  try
  {
    private_nh_.getParam(key, param);
    for (size_t i = 0; i < size; i++)
    {
      for (size_t j = 0; j < size; j++)
      {
        // needed if all points don't have decimal points
        std::ostringstream os;
        os << param[size * i + j];
        std::istringstream is(os.str());
        is >> mat(i, j);
      }
    }
  }
  catch (...)
  {
    ROS_ERROR("Error loading %s param", "initial_estimate_covariance");
  }
}

tf2::Transform LocalizationRos::getTransformFrame(
    const std_msgs::Header &header)
{
  tf2::Transform targetFrameTrans;
  try
  {
    tf2::fromMsg(tfBuffer_
                     .lookupTransform(baseLinkFrame_, header.frame_id,
                                      header.stamp, ros::Duration(0.01))
                     .transform,
                 targetFrameTrans);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from "
                                      << header.frame_id << " to "
                                      << baseLinkFrame_
                                      << ". Error: " << ex.what());
  }
  return targetFrameTrans;
}

void LocalizationRos::clearMeasurementQueue()
{
  while (!measurementQueue_.empty() && ros::ok())
  {
    measurementQueue_.pop();
  }
}

void LocalizationRos::rotateCovariance(const double *covariance,
                                       const tf2::Quaternion &q,
                                       Eigen::MatrixXd &rotated)
{
  // create Eigen matrix with rotation q
  tf2::Matrix3x3 tfRot(q);
  Eigen::MatrixXd rot(3, 3);
  for (size_t i = 0; i < 3; ++i)
  {
    rot(i, 0) = tfRot.getRow(i).getX();
    rot(i, 1) = tfRot.getRow(i).getY();
    rot(i, 2) = tfRot.getRow(i).getZ();
  }
  // copy covariance to rotated
  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = 0; j < 3; j++)
    {
      rotated(i, j) = covariance[3 * i + j];
    }
  }
  rotated = rot * rotated.eval() * rot.transpose();
}
