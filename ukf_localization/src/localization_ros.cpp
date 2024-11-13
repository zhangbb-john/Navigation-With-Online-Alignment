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
    // baseLinkFrame_ = ukf_localization::load_frame("robot");
    //  todo: remove temp output frame
    //     odomFrame_ = au_core::load_frame("odom");
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

    gpsInfoPub_ = nh_.advertise<auv_nav_msg::GPSInfo>("/NavLog/GPSINFO", 20);
    gpsSub_ = nh_.subscribe("/Sensor/Gps", 5, &LocalizationRos::gpsCallback, this);

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
  // time_record = 0;
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
    // ROS_INFO("measurement is not empty");
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
        // deltaT = 0.01;
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
      // deltaT = 0.01;
    }

    if (count_no_sensor <= 5)
    {
      // timefile<<deltaT<<std::endl;
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
  state_topic = getStateMessage();
  statePub_.publish(state_topic);

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
  odom.pose.pose.position.x = state(StateX) - odom_vs_state_x;
  odom.pose.pose.position.y = state(StateY) - odom_vs_state_y;
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

  // ROS_INFO("odom angular x: %f, y: %f, z: %f,w:%f roll:%f, pitch:%f, yaw:%f,vx:%f, vy:%f,vz:%f",odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w,state(StateRoll), state(StatePitch), state(StateYaw),state(StateVx),state(StateVy),state(StateVz) );
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
  statefile << std::fixed << std::setprecision(5) << ros::Time::now().toSec() - start_time << " " << stateVector << std::endl;
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
  Eigen::Vector3d motion(stateVector(StateX) - last_pos.x(), stateVector(StateY) - last_pos.y(), stateVector(StateZ) - last_pos.z());
  distance += motion.norm();
  last_pos << state.position.x, state.position.y, state.position.z;

  return state;
}
void LocalizationRos::publishOdomTf()
{
  // publish odom message
  // odomfile << std::fixed << std::setprecision(3) <<filteredState.header.stamp.toSec()-start_time<<" "<< filteredState.pose.pose.position.x<<" "<<filteredState.pose.pose.position.y<<" "<<filteredState.pose.pose.position.z<<" "<<filteredState.twist.twist.linear.x<<" "<<filteredState.twist.twist.linear.y<<" "<<filteredState.twist.twist.linear.z<<std::endl;
  // ROS_INFO("odom published \n stamp: %f", odomTransMsg.header.stamp.toSec());
  // ROS_INFO("[ x: %f y: %f z: %f]", odomTransMsg.transform.translation.x,odomTransMsg.transform.translation.y,odomTransMsg.transform.translation.z );
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
#ifdef USE_FIXED_ORIGIN
    std::cout << "is_gps_init is " << is_gps_init << std::endl;
    if (is_gps_init)
      odomPub_.publish(filteredState);
#else
    odomPub_.publish(filteredState);
#endif
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
  // std::cout<<"LocalizationRos::clockCallback: time record is "<<time_record<<std::endl;
}
#endif

#if IMU_MSG == AHRS
void LocalizationRos::imuCallback(const cola2_msgs::AHRSConstPtr &msg)
#elif IMU_MSG == SENSOR_MSG_IMU
void LocalizationRos::imuCallback(const sensor_msgs::ImuConstPtr &msg)
#else
void LocalizationRos::imuCallback(const sensor_msgs::ImuConstPtr &msg)
#endif
{
  // std::cout<<"received"<<std::endl;
  //  double imu_time = msg->header.stamp.toSec();
  double imu_time = ros::Time::now().toSec();
  if (imu_time - last_imu_time >= 0.04)
  {
    last_imu_time = imu_time;
    if (msg->orientation_covariance[0] > 0)
    {
      MeasurementPtr z = std::make_shared<Measurement>(IMU_SIZE);
      z->type = MeasurementTypeImu;
      z->time = ros::Time::now().toSec(); // msg->header.stamp.toSec();
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
      orientation.getRPY(roll, pitch, yaw);
      z->measurement(ImuRoll) = roll;
      z->measurement(ImuPitch) = pitch;
      z->measurement(ImuYaw) = yaw;
#ifdef MAG_ATT
      Eigen::Vector3d magnetic_field, offset;
      Eigen::Matrix3d Matrix;
      Matrix << 2.30143105814, -0.0119265620537, 0.0313647834363, 
              -0.0119265620537, 2.31043275104, -0.0203522843, 
              0.0313647834363, -0.0203522843, 3.07785507067;
      offset << 0.0176434323973, 0.0481479974883, 0.801803148557;
      //recent one 
      // Matrix <<  2.24519761547, 0.0522175002349, 0.0342726064562, 
      // 0.0522175002349, 2.26946399256, 0.0237359860959, 
      // 0.0342726064562, 0.0237359860959, 2.92277496654, 
      // offset << 0.0155037728652, 0.0552260424296, 0.782281948329, 
      magnetic_field = Matrix * (Eigen::Vector3d(msg->magnetic_field[0], msg->magnetic_field[1], msg->magnetic_field[2]) - offset); 
      
      double mag_r, mag_p, mag_y;

      mag_r = atan2(-msg->linear_acceleration.y, -msg->linear_acceleration.z);
      mag_p = atan2(msg->linear_acceleration.x, - msg->linear_acceleration.z / cos(mag_r));
      double mx, my;
      mx = cos(mag_p) * magnetic_field[0] + sin(mag_p) * sin(mag_r) * (magnetic_field[1]) + sin(mag_p) * cos(mag_r) * (magnetic_field[2]);
      my = cos(mag_r) * (magnetic_field[1]) - sin(mag_r) * (magnetic_field[2]);
      mag_y = atan2(-my, mx);
      z->measurement(ImuRoll) = mag_r;
      z->measurement(ImuPitch) = mag_p;
      z->measurement(ImuYaw) = mag_y;
#endif


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
      z->covariance.block<3, 3>(ImuVroll, ImuVroll) = 9 * covarianceRotated;
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
      // logfile<<"covariance for imu callback:"<<std::endl;
      Eigen::MatrixX3d covarianceMatrix = Eigen::Vector3d(msg->orientation_covariance[0],
                                                          msg->orientation_covariance[4], msg->orientation_covariance[8])
                                              .asDiagonal();
      // logfile<<covarianceMatrix<<std::endl;
      //  rotate covariance matrix to base_link
      rotateCovariance(&(msg->linear_acceleration_covariance[0]),
                       targetFrameTrans.getRotation(), covarianceRotated);
      z->covariance.block<3, 3>(ImuAx, ImuAx) = 9 * covarianceRotated;

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
  z->time = ros::Time::now().toSec(); // msg->header.stamp.toSec();

  // std::cout<< "#ifdef BAG/dvlCallback: time_record is "<<time_record<<std::endl;

  // #else
  // z->time = ros::Time::now().toSec();
  // std::cout<< "#else /dvlCallback: time_record is "<<z->time<<std::endl;

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

    Eigen::Vector3d noise_vec(0, 0, 0);

    double error = params_.dvl_noise_sigma;

    if (ros::Time::now().toSec() - start_time > 500)
    {
      //add additional noise
      std::random_device rd;
      std::default_random_engine generator_(rd());
      std::normal_distribution<double> noise(0.0, 1.0);
      noise_vec << noise(generator_) * ADD_NOISE, noise(generator_) * ADD_NOISE, noise(generator_) * ADD_NOISE;
      error = params_.dvl_noise_sigma + ADD_NOISE;

    }
    z->measurement(0) = linVel.x() / 1000 + noise_vec(0);
    z->measurement(1) = linVel.y() / 1000 + noise_vec(1);
    z->measurement(2) = linVel.z() / 1000 + noise_vec(2);
    // rotate covariance matrix to base_link

    double covariance[9] = {error * error, 0, 0, 0, error * error, 0, 0, 0, error * error};
    // rotateCovariance(&(msg->velocity_covariance[0]),
    //                  targetFrameTrans.getRotation(), covarianceRotated);
    Eigen::MatrixXd covarianceRotated(3, 3);   

    rotateCovariance(&(covariance[0]),
                     targetFrameTrans.getRotation(), covarianceRotated);
    z->covariance.block<3, 3>(0, 0) = covarianceRotated;

    // measure_log<<"dvl"<<std::endl;
    // dvlfile << std::fixed << std::setprecision(3) << "received velocity :" << " "<<" time :"<<z->time <<" " << z->measurement(0)<<" "<<z->measurement(1) <<   " "<<z->measurement(2)<<std::endl;

    // std::cout<<"dvl time is "<<z->time<<std::endl;
    measurementQueue_.push(z);
  }
}

void LocalizationRos::depthCallback(const sensor_msgs::FluidPressureConstPtr &msg)
{
  MeasurementPtr z = std::make_shared<Measurement>(DEPTH_SIZE);
  z->type = MeasurementTypeDepth;
  z->time = ros::Time::now().toSec(); // msg->header.stamp.toSec(); //
  // tf2::Transform targetFrameTrans = getTransformFrame(msg->header);
  tf2::Transform targetFrameTrans;
  targetFrameTrans.setIdentity();
  // take into account positional offset of depth sensor
  z->measurement(0) = msg->fluid_pressure - targetFrameTrans.getOrigin().z();
  // z->covariance(0, 0) = msg->variance;
  z->covariance(0, 0) = params_.depth_noise_sigma * params_.depth_noise_sigma;

  // std::cout<<"depth time is "<<z->time<<std::endl;
  //  std::cout << "depth received" << std::endl;
  measurementQueue_.push(z);
}

void LocalizationRos::usbllongCallback(const auv_nav_msg::USBLLONGConstPtr &msg)
{
  const Eigen::VectorXd &state = filter_.getState();

#ifdef FAKE_RECVIM
  MeasurementPtr fake_recvim = std::make_shared<Measurement>(RECVIM_SIZE);
  fake_recvim->type = MeasurementTypeRecvim;
  fake_recvim->time = ros::Time::now().toSec(); // msg->header.stamp.toSec();
  // field test
  double depth = 1.2;
  double depth_sigma = 1.0;
  fake_recvim->measurement << 0, depth;
  double speed_error = 50; // 0.03;
#if METHOD == PROPOSED
  double range = sqrt(msg->pos_xyz.x * msg->pos_xyz.x + msg->pos_xyz.y * msg->pos_xyz.y + msg->pos_xyz.z * msg->pos_xyz.z);
  Eigen::Vector3d dir_body2beacon_in_body(msg->pos_xyz.x / range, msg->pos_xyz.y / range, msg->pos_xyz.z / range);
  Eigen::Vector3d vel_body(state(StateVx), state(StateVy), state(StateVz));
  double doppler = vel_body.transpose() * dir_body2beacon_in_body;
  fake_recvim->measurement << doppler, depth;
  speed_error = 0.05;
#endif

  fake_recvim->covariance << speed_error * speed_error, 0,
      0, depth_sigma * depth_sigma;
  measurementQueue_.push(fake_recvim);
#endif

#ifdef ADJUST_NOISE
   
  std::cout << "max_bearing is " << max_bearing << "; min_bearing is " << min_bearing << std::endl;
  if (normalizeAngle(max_bearing - min_bearing) > 1.5)
  {
    std::cout << "set new process noise" << std::endl;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q = params_.processNoiseCov;
    Q.block<3, 3>(StateBeaconX, StateBeaconX) = Eigen::Vector3d(1.0e-5, 1.0e-5, 1.0e-5).asDiagonal();
    filter_.changeProcessNoise(Q);
  }
#endif
  if (msg->accuracy < 20)
  {
    MeasurementPtr z = std::make_shared<Measurement>(ANGLE_SIZE);
    z->type = MeasurementTypeAngle;
    // z->time = ros::Time::now().toSec() - (msg->current_time - msg->measurement_time);//msg->header.stamp.toSec() - (msg->current_time - msg->measurement_time);
    z->time = ros::Time::now().toSec();
    double dist = sqrt(msg->pos_xyz.x * msg->pos_xyz.x + msg->pos_xyz.y * msg->pos_xyz.y + msg->pos_xyz.z * msg->pos_xyz.z);
    double bearing = atan2(msg->pos_xyz.y, msg->pos_xyz.x);
    double elevation = asin(msg->pos_xyz.z / dist);
    z->measurement << bearing + MISALIGN, elevation;
    double error = 10 * asin(msg->accuracy / dist);
    std::cout << "In USBLLONG, the error is " << error << std::endl;
    logfile << "USBLLONG, bearing is " << bearing << "; elevation is " << elevation << std::endl;
    z->covariance << error * error, 0,
        0, error * error * 25;
    bool InLier = true;

    Eigen::Vector3d pos_enu;
    pos_enu << msg->pos_enu.x, msg->pos_enu.y, msg->pos_enu.z;

    InLier = (state(StateY) + pos_enu.x() > 3) && (state(StateY) + pos_enu.x() < 31) && (state(StateX) + pos_enu.y() > -29) && (state(StateX) + pos_enu.y() < -14) && (-state(StateZ) + pos_enu.z() > -12) && (-state(StateZ) + pos_enu.z() < 5);
    Eigen::Vector3d beacon_pos;
    beacon_pos << state(StateY) + pos_enu.x(), state(StateX) + pos_enu.y(), -state(StateZ) + pos_enu.z();
    std::cout << "beacon_pos is " << beacon_pos.transpose() << std::endl;
    beaconfile << beacon_pos.transpose() << std::endl;

    if (InLier) //&& state(StateZ) > 4)
    {
      double azimuth = atan2(msg->pos_enu.x, msg->pos_enu.y);
      std::cout << "azimuth is " << azimuth << std::endl;
      if (azimuth < 0)
      {
        azimuth = azimuth + 2 * M_PI;
      }
      if(azimuth < min_bearing)
      {
        min_bearing = azimuth;
      }
      if(azimuth > max_bearing)
      {
        max_bearing = azimuth;
      }
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
#ifdef FAKE_RECVIM
  MeasurementPtr fake_recvim = std::make_shared<Measurement>(RECVIM_SIZE);
  fake_recvim->type = MeasurementTypeRecvim;
  fake_recvim->time = ros::Time::now().toSec(); // msg->header.stamp.toSec();
  // field test
  double depth = 1.2;
  double depth_sigma = 1.0;
  fake_recvim->measurement << 0, depth;
  double speed_error = 50; // 0.03;
  fake_recvim->covariance << speed_error * speed_error, 0,
      0, depth_sigma * depth_sigma;
  measurementQueue_.push(fake_recvim);
#endif
  logfile << "received angle; now time in second is " << ros::Time::now().toSec() - start_time << std::endl;

  std::cout << "received angle; now time in second is " << ros::Time::now().toSec() - start_time << std::endl;
#ifdef ADJUST_NOISE
  if (normalizeAngle(max_bearing - min_bearing) > 1.5)
  {
    std::cout << "set new process noise" << std::endl;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q = params_.processNoiseCov;
    Q.block<3, 3>(StateBeaconX, StateBeaconX) = Eigen::Vector3d(1.0e-5, 1.0e-5, 1.0e-5).asDiagonal();
    filter_.changeProcessNoise(Q);
  }
#endif
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

    error = msg->accuracy * 2;
    std::cout << "now time in second is " << ros::Time::now().toSec() - start_time << std::endl;
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
  z->time = ros::Time::now().toSec(); // msg->header.stamp.toSec();
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
  double speed_error = 0.3; // 0.03;
  speed_error = params_.recvim_noise_sigma;

  z->covariance << speed_error * speed_error, 0,
      0, depth_sigma * depth_sigma;
  measurementQueue_.push(z);
}

void LocalizationRos::gpsCallback(const cola2_msgs::GPS msg)
{
  int sum_snr = 0;
  double Re = 6378137, f = 1 / 298.257, e = sqrt(2 * f - f * f);
  // haihong 1 ocean
  Eigen::Vector3d Gps2Imu(0.55, 0.55, 0.35);
  // haihong 2 Eigen::Vector3d Gps2Imu(0.69, 0.16, 0.5); //(0, 0, 0); //
  // calculate the ccovariance of gps fix
  for (int i = 0; i < msg.satellite_visible_snr.size(); i++)
  {
    sum_snr = sum_snr + msg.satellite_visible_snr[i];
  }
  int mean_snr = 0;
  if (msg.satellite_visible_snr.size() > 0)
  {
    mean_snr = sum_snr / msg.satellite_visible_snr.size();
  }
  // std::cout << "msg.satellite_visible_snr.size() is " << msg.satellite_visible_snr.size() << std::endl;
  // logfile << "GPS SNR is " << mean_snr << "; status is " << msg.status << std::endl;
  // mean_snr = 28;
  double covariance = 1.0;
  if (msg.latitude <= 0 || msg.satellites_used < 11 || mean_snr < 25)
  {
    covariance = 1e8;
  }
  else if (msg.status == 4)
  {
    covariance = 0.04;
  }
  else if (mean_snr > 35)
  {
    covariance = 4;
  }
  else
  {
    covariance = 100;
  }
  // std::cout << "covariance is " << covariance << std::endl;

  // action associated with covariance
  if (covariance > 100)
  {
    logfile << "Wrong GPS" << std::endl;
    logfile << "covariance is " << covariance << std::endl;
    logfile << "msg.latitude is " << msg.latitude << "; msg.satellites_used is " << msg.satellites_used << "; mean_snr: " << mean_snr << std::endl;
    return;
  }
  Eigen::Matrix<double, STATE_SIZE, 1> state_vec;
  state_vec = filter_.getState();
  Eigen::Matrix3d W2IMU = (Eigen::AngleAxisd(state_vec(StateYaw), Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(state_vec(StatePitch), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(state_vec(StateRoll), Eigen::Vector3d::UnitX()))
                              .toRotationMatrix();

  if (!is_gps_init)
  {
    logfile << "Good GPS" << std::endl;
    logfile << "filter_.isInitialized() is " << filter_.isInitialized() << std::endl;
    if (filter_.isInitialized() && covariance < 1)
    {

      Eigen::Vector3d InitialGps2Imu;
      InitialGps2Imu = W2IMU * Gps2Imu; // Eigen::Vector3d(0.55, 0.35, 0.55);
      logfile << "W2IMU is " << std::endl
              << W2IMU << "; InitialGps2Imu is " << std::endl
              << InitialGps2Imu << std::endl;

      double current_gps_lat = msg.latitude / 180 * M_PI, current_gps_lon = msg.longitude / 180 * M_PI;
      double Rn = Re / sqrt(1 - e * e * sin(current_gps_lat) * sin(current_gps_lat));
      double Rm = Rn * (1 - e * e) / (1 - e * e * sin(current_gps_lat) * sin(current_gps_lat));
      double current_imu_lat = InitialGps2Imu.x() / Rm + current_gps_lat;
      double current_imu_lon = InitialGps2Imu.y() / (Rn * cos(current_imu_lat)) + current_gps_lon;
      gps_start_lat = (-state_vec(StateX)) / Rm + current_imu_lat;
      gps_start_lon = (-state_vec(StateY)) / (Rn * cos(current_imu_lat)) + current_imu_lon;
      last_pos << state_vec(StateX), state_vec(StateY), 0;

      // gps_start_lat = msg.latitude / 180 * M_PI;
      // gps_start_lon = msg.longitude / 180 * M_PI;

      last_GPS_time = ros::Time::now().toSec();
      logfile << std::fixed << std::setprecision(11) << "GPS_start_x: " << state_vec(StateX) << "; GPS_start_y: " << state_vec(StateY) << "msg.latitude, msg.longitude is " << msg.latitude << ";" << msg.longitude << "gps_start_lat, gps_start_lon is " << gps_start_lon << ", " << gps_start_lon << std::endl;
      // Eigen::Matrix<double, 4, 1> init_gps(GPS_start_x, GPS_start_y, msg.latitude, msg.longitude);
      is_gps_init = true;
      distance = 0;
      t_start = ros::Time::now().toSec();
      last_pos << state_vec(StateX), state_vec(StateY), 0;
      auv_nav_msg::GPSInfo gps_info;
      gps_info.header.stamp = ros::Time::now();
      gps_info.pos_ned.x = state_vec(StateX);
      gps_info.pos_ned.y = state_vec(StateY);
      gps_info.pos_ned.z = 0;
      gps_info.pos_err.x = 0;
      gps_info.pos_err.y = 0;
      gps_info.pos_err.z = 0;
      gps_info.uncertainty = sqrt(covariance);
      gps_info.deviation = 0;
      gps_info.distance = distance;
      gps_info.accuracy = 0;
      gps_info.duration = ros::Time::now().toSec() - t_start;
      gpsInfoPub_.publish(gps_info);

#ifdef USE_FIXED_ORIGIN
      // fixed origin
      odom_vs_state_x = Rm * (fixed_lat / 180.0 * M_PI - gps_start_lat);
      odom_vs_state_y = Rn * cos(gps_start_lat) * (fixed_lon / 180.0 * M_PI - gps_start_lon);
#endif
      // associated with beacon
      double lat_beacon; //= 0.525493468;
      double lon_beacon; //= 2.1291601054;
      lat_beacon = 30.1086092437 / 180.0 * M_PI;
      lon_beacon = 121.991890894 / 180.0 * M_PI;
      double londis = 5;   // Re * cos(lat_beacon) * (lon_beacon - gps_start_lon) * (1 + f * sin(lat_beacon) * sin(lat_beacon));
      double latdis = -25; // Re * ((lat_beacon - gps_start_lat) * (1 - 2 * f + 3 * f * 0.5) -
                           //  3 * f * 0.25 * (sin(2 * lat_beacon) - sin(2 * gps_start_lat)));
      beaconPos << 0, 0, 0.5;
      // beaconPos << -15.9825, 5.99791, 0.493763;
      std::cout << "beaconPos is " << beaconPos << std::endl;
      // beacon position is determined by latitude and longitude
      state_vec.segment<2>(StateBeaconX) << latdis, londis;
      filter_.setState(state_vec);
    }
  }
  else
  {
    // take into account positional offset of depth sensor
    double lat = msg.latitude / 180 * M_PI;
    double lon = msg.longitude / 180 * M_PI;
    double Rn = Re / sqrt(1 - e * e * sin(gps_start_lat) * sin(gps_start_lat));
    double Rm = Rn * (1 - e * e) / (1 - e * e * sin(gps_start_lat) * sin(gps_start_lat));
    double latdis = Rm * (lat - gps_start_lat), londis = Rn * cos(gps_start_lat) * (lon - gps_start_lon);

    double gap = ros::Time::now().toSec() - last_GPS_time;
    std::cout << "gap is " << gap << std::endl;

    if (gap > 0.5)
    {
      last_GPS_time = ros::Time::now().toSec();
      Eigen::Vector3d tGps2Imu;
      tGps2Imu = W2IMU * Gps2Imu; // Eigen::Vector3d(0.55, 0.35, 0.55);//1.2, 0, 0.5
      // std::cout << "W2IMU is " << W2IMU << std::endl;

      // std::cout<<"depth time is "<<z->time<<std::endl;
      // std::cout << "gps received" << std::endl;
      double gps_north = latdis + tGps2Imu.x();
      double gps_east = londis + tGps2Imu.y();
      logfile << "W2IMU is " << std::endl
              << W2IMU << "; tGps2Imu is " << std::endl
              << tGps2Imu << std::endl;
      MeasurementPtr z = std::make_shared<Measurement>(XY_SIZE);
      z->type = MeasurementTypeXY;
      z->time = ros::Time::now().toSec(); // msg->header.stamp.toSec();
      z->measurement(0) = gps_north;
      z->measurement(1) = gps_east;
      // debug
      z->covariance << covariance, 0,
          0, covariance;
      double deviation = sqrt(pow((gps_north - state_topic.position.x), 2) + pow((gps_east - state_topic.position.y), 2));
      // GPSINFO publisher
      auv_nav_msg::GPSInfo gps_info;
      gps_info.header.stamp = ros::Time::now();
      gps_info.pos_ned.x = gps_north;
      gps_info.pos_ned.y = gps_east;
      gps_info.pos_ned.z = 0;
      gps_info.pos_err.x = gps_north - state_topic.position.x;
      gps_info.pos_err.y = gps_east - state_topic.position.y;
      gps_info.pos_err.z = 0;
      gps_info.uncertainty = sqrt(covariance);
      gps_info.deviation = deviation;
      gps_info.distance = distance;
      gps_info.accuracy = deviation / distance;
      gps_info.duration = ros::Time::now().toSec() - t_start;
      gpsInfoPub_.publish(gps_info);
      // Log file
      logfile << " deviation: " << deviation << "; 10 *sqrt(state_topic.position_covariance_diag.x + state_topic.position_covariance_diag.x + 1):" << 10 * sqrt(state_topic.position_covariance_diag.x + state_topic.position_covariance_diag.x + 1) << std::endl;
      if (deviation < 20 * sqrt(state_topic.position_covariance_diag.x + state_topic.position_covariance_diag.y + 1))
      {
        logfile << "gps ok" << std::endl;
#ifdef USE_GPS
        logfile << "gps push measurement" << std::endl;
        measurementQueue_.push(z);
#endif
      }
      else
      {
        logfile << "gps not ok" << std::endl;
      }
    }
  }
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
