#include <localization.h>
#include <fstream>
#include <sys/stat.h>
#include <iomanip> // std::setprecision()

Localization::Localization(double alpha, double beta, double kappa)
    : filter_(alpha, beta, kappa,
              std::bind(&Localization::stateTransitionFunction, this,
                        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)),
      isInitialized_(false),
      isUSBLInitialized_(false),
      lastMeasurementTime_(0.0)
{
  // setup process and measurement models
  setupProcessModel();
  setupMeasurementModels();

  // process noise (can be reset by setProcessNoise)
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q;
  Q.setZero();
  // Q.diagonal() << 0.05, 0.05, 0.05, 0.03, 0.03, 0.06, 0.025, 0.025, 0.025, 0.01,
  //     0.01, 0.02, 0.01, 0.01, 0.01;
  Q.diagonal() << 0.05, 0.05, 0.05, 0.01, 0.01, 0.01, 0.025, 0.025, 0.025, 0.01,
      0.01, 0.02, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.03;
  filter_.setProcessNoise(Q);
  // initial estimate covariance (can be reset by setInitialCovariance)
  initialP_.setIdentity();
  initialP_ *= 1e-9;

  reset();
}

void Localization::reset()
{
  transitionMatrix_.setIdentity();
  filter_.setCovariance(initialP_);
  // logfile<<filter_.getCovariance()<<std::endl;
  filter_.setState(Eigen::Matrix<double, STATE_SIZE, 1>::Zero());

  lastMeasurementTime_ = 0.0;
  isInitialized_ = false;
  isUSBLInitialized_ = !INITIALIZATION_NLS_;
  std::cout << "Reset localization class, now isUSBLInitialized_ is " << isUSBLInitialized_  << " while  INITIALIZATION_NLS is " << INITIALIZATION_NLS_<< std::endl;
  std::cout << "Get threshold during reset: " << initial_usbl.get_threshold() << std::endl;
}

void Localization::predict(double deltaT) { filter_.predict(deltaT); }
void Localization::processMeasurement(Measurement measurement)
{
  double deltaT = 0.0;
  // std::cout << "get threshold when receiving measurement "<<initial_usbl.get_threshold() << std::endl;

  //measurefile<<"now start process measurement"<<std::endl;
  // std::cout << " measurement.time is "<<measurement.time<<std::endl;
  if (isInitialized_)
  { // predict + update cycle
    deltaT = measurement.time - lastMeasurementTime_;

    if (deltaT > 100000.0)
    {
      std::cout << "Delta was very large. Suspect playing from bag file. "
                   "Setting to 0.01"
                << std::endl;
    }
    else if (deltaT < 0.0)
    {
      std::cout << "Received old reading. Skipping!"
                << " measurement.time is " << measurement.time << " and lastMeasurementTime_ is " << lastMeasurementTime_ << std::endl;
      // logfile << "deltaT < 0.0"<<std::endl;
      timefile << " 00 ";
      return;
    }
    //timefile << " 22 ";
    // logfile<<" (deltaT > 0.0) and" <<std::endl;
    // predict update cycle for the measurement
    logfile << "old state:" << std::endl
            << filter_.getState() << std::endl
            << std::endl;
    //std::cout<<"deltaT is "<<deltaT<<std::endl;
    filter_.predict(deltaT);
    // Eigen::Matrix3d Rotation =
    //   (Eigen::AngleAxisd(filter_.getState()(StateYaw), Eigen::Vector3d::UnitZ()) *
    //    Eigen::AngleAxisd(filter_.getState()(StatePitch), Eigen::Vector3d::UnitY()) *
    //    Eigen::AngleAxisd(filter_.getState()(StateRoll), Eigen::Vector3d::UnitX()))
    //       .toRotationMatrix();  // rotation matrix (ZYX)
    //rotfile <<std::fixed<<std::setprecision(5)<< "measurement.time:" <<measurement.time<<std::endl<< filter_.getState()(StateRoll) <<std::endl;
    // rotfile <<std::fixed<<std::setprecision(5)<<measurement.time<<" "<< filter_.getState()(StateRoll)<<" "<<filter_.getState()(StatePitch)<<" "<<filter_.getState()(StateYaw) <<std::endl;

    logfile << "predict state:" << std::endl
            << filter_.getState() << std::endl
            << std::endl;
    logfile <<std::fixed<<std::setprecision(2) << "measurement time:" << measurement.time - startSec << std::endl;

    //measurefile<<"isInitialized_:now start process measurement"<<std::endl;
    if (measurement.type == MeasurementTypeDepth)
    {
      Eigen::Matrix<double, DEPTH_SIZE, 1> z = measurement.measurement;
      Eigen::Matrix<double, DEPTH_SIZE, DEPTH_SIZE> R = measurement.covariance;
      //measurefile<<"depth"<<std::endl;
      logfile << "depth" << std::endl;
      //measurefile<<measurement.measurement<<std::endl;
      filter_.update(z, R, depthModel_);
    }
    else if (measurement.type == MeasurementTypeDvl)
    {
      Eigen::Matrix<double, DVL_SIZE, 1> z = measurement.measurement;
      Eigen::Matrix<double, DVL_SIZE, DVL_SIZE> R = measurement.covariance;
      //measurefile<<"Dvl"<<std::endl;
      logfile <<std::fixed<<std::setprecision(2) << "Dvl"
              << ": time is " << measurement.time << std::endl;
      //measurefile<<measurement.measurement<<std::endl;
      filter_.update(z, R, dvlModel_);
    }
    else if (measurement.type == MeasurementTypeImu)
    {
      Eigen::Matrix<double, IMU_SIZE, 1> z = measurement.measurement;
      Eigen::Matrix<double, IMU_SIZE, IMU_SIZE> R = measurement.covariance;
      //measurefile<<"Imu"<<std::endl;
      logfile <<std::fixed<<std::setprecision(2) << "Imu"
              << ": time is " << measurement.time << std::endl;
      //measurefile<<measurement.measurement<<std::endl;
      filter_.update(z, R, imuModel_);
    }
    else if (measurement.type == MeasurementTypeAngle)
    {
      Eigen::Matrix<double, ANGLE_SIZE, 1> z = measurement.measurement;
      Eigen::Matrix<double, ANGLE_SIZE, ANGLE_SIZE> R = measurement.covariance;
      //measurefile<<"Imu"<<std::endl;
      logfile <<std::fixed<<std::setprecision(2) << "angle"
              << ": time is " << measurement.time << std::endl;
      //measurefile<<measurement.measurement<<std::endl;
      Eigen::Vector3d beacon_pos;
      Eigen::Vector3d usbl_rpy;
      if (isUSBLInitialized_)
      {
        filter_.update(z, R, angleModel_);
        logfile << "isUSBLInitialized_: true; update with angle" << std::endl;
      }
      else 
      {
        AngleMeasurement angle_measurement;
        angle_measurement.angle(0) = z(0);
        angle_measurement.angle(1) = z(1);
        angle_measurement.sigma(0) = sqrt(R(0, 0));
        angle_measurement.sigma(1) = sqrt(R(1, 1));
        Eigen::Matrix<double, STATE_SIZE, 1> x = filter_.getState();
        angle_measurement.position << x(StateX), x(StateY), x(StateZ);
        Eigen::Quaterniond q = Eigen::AngleAxisd(-x(StateRoll), Eigen::Vector3d::UnitX()) *  Eigen::AngleAxisd(-x(StatePitch), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-x(StateYaw), Eigen::Vector3d::UnitZ());
        // Eigen::Quaterniond q_w2base = Eigen::AngleAxisd(x(StateYaw), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(x(StatePitch), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(x(StateRoll), Eigen::Vector3d::UnitX());
        // Eigen::Matrix3d R_w2base(q_w2base);
        // std::cout << "q to matrix " << std::endl << q.matrix() << std::endl << "R_w2base" << std::endl << R_w2base.transpose() << std::endl << std::endl;
        angle_measurement.q = q;
        angle_measurements.push_back(angle_measurement);
        logfile << "push back angle measurement" << std::endl;

        isUSBLInitialized_ = initial_usbl.Initialization(angle_measurements, recvim_measurements, beacon_pos, usbl_rpy);
        if (isUSBLInitialized_)
        {
          std::cout << "beacon_pos: " << beacon_pos << std::endl << "usbl_rpy:" << usbl_rpy << std::endl;
          Eigen::Matrix<double, USBL_INITIAL_SIZE, 1> initial_state;
          initial_state << beacon_pos[0], beacon_pos[1], beacon_pos[2], usbl_rpy[0], usbl_rpy[1], usbl_rpy[2];
          Eigen::Matrix<double, USBL_INITIAL_SIZE, USBL_INITIAL_SIZE> InitialCovariance;
          InitialCovariance <<  1.0e-3, 0,      0,      0,      0,      0,
                                0,      1.0e-3, 0,      0,      0,      0,
                                0,      0,      1.0e-3, 0,      0,      0,
                                0,      0,      0,      1.0e-4, 0,      0,
                                0,      0,      0,      0,      1.0e-4, 0,
                                0,      0,      0,      0,      0,      1.0e-4;
                                                                                           

          filter_.update(initial_state, InitialCovariance, usblInitialModel_);
        }
      }
    }
    else if (measurement.type == MeasurementTypeRecvim)
    {
      Eigen::Matrix<double, RECVIM_SIZE, 1> z = measurement.measurement;
      Eigen::Matrix<double, RECVIM_SIZE, RECVIM_SIZE> R = measurement.covariance;

      //measurefile<<"Imu"<<std::endl;
      logfile << "Recvim"
              << ": time is " << measurement.time << std::endl;
      //measurefile<<measurement.measurement<<std::endl;
      Eigen::Vector3d beacon_pos;
      Eigen::Vector3d usbl_rpy;
      if (isUSBLInitialized_)
        filter_.update(z, R, recvimModel_);
      else 
      {
        RecvimMeasurement recvim_measurement;
        recvim_measurement.recvim(0) = z(0);
        recvim_measurement.recvim(1) = z(1);

        recvim_measurement.sigma(0) = sqrt(R(0, 0));
        recvim_measurement.sigma(1) = sqrt(R(1, 1));

        Eigen::Matrix<double, STATE_SIZE, 1> x = filter_.getState();
        recvim_measurement.position << x(StateX), x(StateY), x(StateZ);
        Eigen::Quaterniond q = Eigen::AngleAxisd(-x(StateRoll), Eigen::Vector3d::UnitX()) *  Eigen::AngleAxisd(-x(StatePitch), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-x(StateYaw), Eigen::Vector3d::UnitZ());
        recvim_measurement.q = q;
        recvim_measurement.velocity << x(StateVx), x(StateVy), x(StateVz);
        recvim_measurements.push_back(recvim_measurement);

        isUSBLInitialized_ = initial_usbl.Initialization(angle_measurements, recvim_measurements, beacon_pos, usbl_rpy);
        if (isUSBLInitialized_)
        {
          std::cout << "beacon_pos: " << beacon_pos << std::endl << "usbl_rpy:" << usbl_rpy << std::endl;
          Eigen::Matrix<double, USBL_INITIAL_SIZE, 1> initial_state;
          initial_state << beacon_pos[0], beacon_pos[1], beacon_pos[2], usbl_rpy[0], usbl_rpy[1], usbl_rpy[2];
          Eigen::Matrix<double, USBL_INITIAL_SIZE, USBL_INITIAL_SIZE> InitialCovariance;
          InitialCovariance <<  1.0e-2, 0,      0,      0,      0,      0,
                                0,      1.0e-2, 0,      0,      0,      0,
                                0,      0,      1.0e-2, 0,      0,      0,
                                0,      0,      0,      1.0e-4, 0,      0,
                                0,      0,      0,      0,      1.0e-4, 0,
                                0,      0,      0,      0,      0,      1.0e-4;                   

          filter_.update(initial_state, InitialCovariance, usblInitialModel_);
        }
      }
    }

    logfile << "update state:" << std::endl
            << filter_.getState() << std::endl
            << std::endl;

    //measurefile<<measurement.time-startSec<<std::endl;
  }
  else if (measurement.type == MeasurementTypeImu)
  {
    x.setZero();
    P = initialP_;
    std::cout << "not Initialized_:now start process measurement with imu" << std::endl;
    // copy orientation
    x.segment<3>(StateRoll) = measurement.measurement.segment<3>(ImuRoll);
    P.block<3, 3>(StateRoll, StateRoll) =
        measurement.covariance.block<3, 3>(ImuRoll, ImuRoll);
    // copy angular velocity

    x.segment<3>(StateVroll) = measurement.measurement.segment<3>(ImuVroll);
    P.block<3, 3>(StateVroll, StateVroll) =
        measurement.covariance.block<3, 3>(ImuVroll, ImuVroll);
    // copy acceleration
    x.segment<3>(StateAx) = measurement.measurement.segment<3>(ImuAx);

    //add initial state
    // param
    // double ellipse_x = 15;
    // double ellipse_y = 20;
    // double z = 1;           // z轴做sin运动
    // double K1 = 10;          // z轴的正弦频率是x，y的k1倍
    // double K = M_PI/ ( 12 / 2);    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

    // x.segment<3>(StateX) = Eigen::Vector3d( 15 * cos( M_PI / ( 20 / 2 ) * measurement.time ) + 5, 20 * sin( M_PI / (20 / 2) * measurement.time ) + 5,  sin( 10 *  M_PI / ( 20 / 2 ) * measurement.time ) + 5 );
    // x.segment<3>(StateX) = Eigen::Vector3d(0,0,0);
    //x.segment<3>(StateX) = Eigen::Vector3d( ellipse_x * cos( K * measurement.time ) + 5, ellipse_y * sin( K * measurement.time ) + 5,  z * sin( K1 * K * measurement.time ) + 5 );
    //x.segment<3>(StateVx) = Eigen::Vector3d(-0.00000, 4.71239, 0.0000);

    std::cout << "measurement.time" << measurement.time << 15 * cos(M_PI / (20 / 2) * measurement.time) + 5 << " " << 20 * sin(M_PI / (20 / 2) * measurement.time) + 5 << " " << sin(10 * M_PI / (20 / 2) * measurement.time) + 5 << std::endl;

    P.block<3, 3>(StateAx, StateAx) =
        measurement.covariance.block<3, 3>(ImuAx, ImuAx);
    //logfile<<"StateAx, StateAx covariance:"<<std::endl<<measurement.covariance.block<3, 3>(ImuAx, ImuAx)<<std::endl;
    filter_.setState(x);
    // filter_.setCovariance(initialP_);
    filter_.setCovariance(P);
    // logfile<<"initialized covariance:"<<std::endl<<filter_.getCovariance()<<std::endl<<std::endl;
    // logfile<<"initial state:"<<std::endl<<x<<std::endl;
    // logfile<<"initial state for filter :"<<std::endl<<filter_.getState()<<std::endl;
    // std::cout << "3 get threshold when receiving imu inialization "<<initial_usbl.get_threshold() << std::endl;

    std::cout << "Initialized with IMU data" << std::endl;
    isInitialized_ = true;
    // std::cout << "get threshold when receiving imu inialization "<<initial_usbl.get_threshold() << std::endl;

  }
  //std::cout<<"measurement.time is "<<measurement.time<<std::endl;
  if (deltaT >= 0.0)
  {
    lastMeasurementTime_ = measurement.time;
    //std::cout<<"deltaT >= 0 and lastMeasurementTime_ is "<<lastMeasurementTime_<<std::endl;
  }
}
// void Localization::processMeasurement(Measurement measurement) {
//   double deltaT = 0.0;

//   //measurefile<<"now start process measurement"<<std::endl;

//   if (isInitialized_) {  // predict + update cycle
//     deltaT = measurement.time - lastMeasurementTime_;

//     if (deltaT > 100000.0) {
//       std::cout << "Delta was very large. Suspect playing from bag file. "
//                    "Setting to 0.01"
//                 << std::endl;
//       deltaT = 0.01;
//     } else if (deltaT < 0.0) {
//       std::cout << "Received old reading. Skipping!" << std::endl;
//       return;
//     }
//     // predict update cycle for the measurement
//     //logfile<<"old state:"<<std::endl<<filter_.getState()<<std::endl<<std::endl;
//     filter_.predict(deltaT);
//     //logfile<<"predict state:"<<std::endl<<filter_.getState()<<std::endl<<std::endl;
//     //measurefile<<"isInitialized_:now start process measurement"<<std::endl;
//     if (measurement.type == MeasurementTypeDepth) {
//       Eigen::Matrix<double, DEPTH_SIZE, 1> z = measurement.measurement;
//       Eigen::Matrix<double, DEPTH_SIZE, DEPTH_SIZE> R = measurement.covariance;
//       measurefile<<"depth"<<std::endl;
//       logfile<<"depth"<<std::endl;
//       //measurefile<<measurement.measurement<<std::endl;
//       filter_.update(z, R, depthModel_);
//     } else if (measurement.type == MeasurementTypeDvl) {
//       Eigen::Matrix<double, DVL_SIZE, 1> z = measurement.measurement;
//       Eigen::Matrix<double, DVL_SIZE, DVL_SIZE> R = measurement.covariance;
//       measurefile<<"Dvl"<<std::endl;
//       logfile<<"Dvl"<<std::endl;
//       //measurefile<<measurement.measurement<<std::endl;
//       filter_.update(z, R, dvlModel_);
//     } else if (measurement.type == MeasurementTypeImu) {
//       Eigen::Matrix<double, IMU_SIZE, 1> z = measurement.measurement;
//       Eigen::Matrix<double, IMU_SIZE, IMU_SIZE> R = measurement.covariance;
//       measurefile<<"Imu"<<std::endl;
//       logfile<<"Imu"<<std::endl;

//       //measurefile<<measurement.measurement<<std::endl;
//       filter_.update(z, R, imuModel_);
//     }
//     logfile<<"update state:"<<std::endl<<filter_.getState()<<std::endl<<std::endl;

//     measurefile<<measurement.time-startSec<<std::endl;
//   }
//   else {
//     x.setZero();
//     P = initialP_;

//     if (measurement.type == MeasurementTypeDvl){
//       std::cout<<"not Initialized_:now start process measurement with dvl"<<std::endl;
//       x.segment<3>(StateVx) = measurement.measurement.segment<3>(0);
//       P.block<3, 3>(StateVx,StateVx) = measurement.covariance.block<3, 3>(0,0);
//       isDVLInitialized = true;
//     }
//     else if (measurement.type == MeasurementTypeDepth){
//       std::cout<<"not Initialized_:now start process measurement with depth"<<std::endl;
//       x.segment<1>(StateZ) = measurement.measurement.segment<1>(0);
//       P.block<1, 1>(StateZ, StateZ) = measurement.covariance.block<1, 1>(0,0);
//       isDepthInitialized = true;
//     }
//     else if (measurement.type == MeasurementTypeImu && isDVLInitialized == true && isDepthInitialized == true) {
//       std::cout<<"not Initialized_:now start process measurement with imu"<<std::endl;
//       // copy orientation
//       x.segment<3>(StateRoll) = measurement.measurement.segment<3>(ImuRoll);
//       P.block<3, 3>(StateRoll, StateRoll) =
//           measurement.covariance.block<3, 3>(ImuRoll, ImuRoll);
//       // copy angular velocity
//       x.segment<3>(StateVroll) = measurement.measurement.segment<3>(ImuVroll);
//       P.block<3, 3>(StateVroll, StateVroll) =
//           measurement.covariance.block<3, 3>(ImuVroll, ImuVroll);
//       // copy acceleration
//       x.segment<3>(StateAx) = measurement.measurement.segment<3>(ImuAx);
//       //add initial state
//       //x.segment<3>(StateX) = Eigen::Vector3d(0,0,5);
//       //x.segment<3>(StateVx) = Eigen::Vector3d(-0.00000, 4.71239, 0.0000);

//       P.block<3, 3>(StateAx, StateAx) =
//           measurement.covariance.block<3, 3>(ImuAx, ImuAx);
//       //logfile<<"StateAx, StateAx covariance:"<<std::endl<<measurement.covariance.block<3, 3>(ImuAx, ImuAx)<<std::endl;
//       filter_.setState(x);
//       // filter_.setCovariance(initialP_);
//       filter_.setCovariance(P);
//       logfile<<"initialized covariance:"<<std::endl<<filter_.getCovariance()<<std::endl<<std::endl;
//       logfile<<"initial state:"<<std::endl<<x<<std::endl;
//       logfile<<"initial state for filter :"<<std::endl<<filter_.getState()<<std::endl;

//       std::cout << "Initialized with IMU data" << std::endl;
//       isInitialized_ = true;
//     }
//   }

//   if (deltaT >= 0.0) {
//     lastMeasurementTime_ = measurement.time;
//   }
// }

Eigen::Matrix<double, STATE_SIZE, 1> Localization::stateTransitionFunction(
    const Eigen::Matrix<double, STATE_SIZE, 1> &x, double deltaT, bool output)
{
  Eigen::Matrix<double, STATE_SIZE, 1> newState;
  // check readme for full equations
  Eigen::Matrix3d R =
      (Eigen::AngleAxisd(x(StateYaw), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(x(StatePitch), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(x(StateRoll), Eigen::Vector3d::UnitX()))
          .toRotationMatrix(); // rotation matrix (ZYX)

// Eigen::Matrix3d R =
//     (Eigen::AngleAxisd(x(StateRoll), Eigen::Vector3d::UnitX()) *
//      Eigen::AngleAxisd(x(StatePitch), Eigen::Vector3d::UnitY()) *
//      Eigen::AngleAxisd(x(StateYaw), Eigen::Vector3d::UnitZ()))
//         .toRotationMatrix();  // rotation matrix (ZYX)
// transitionMatrix_.block<3, 3>(StateX, StateVx) =  Eigen::Matrix3d::Identity(3, 3) *deltaT;
// transitionMatrix_.block<3, 3>(StateX, StateAx) = Eigen::Matrix3d::Identity(3, 3) *0.5 * deltaT * deltaT;
#ifdef ENU_ACC
  transitionMatrix_.block<3, 3>(StateX, StateVx) = Eigen::Matrix3d::Identity(3, 3) * deltaT;
  transitionMatrix_.block<3, 3>(StateX, StateAx) = 0.5 * Eigen::Matrix3d::Identity(3, 3) * deltaT * deltaT;
#else
  transitionMatrix_.block<3, 3>(StateX, StateVx) = R * deltaT;
  transitionMatrix_.block<3, 3>(StateX, StateAx) = 0.5 * R * deltaT * deltaT;
#endif
  transitionMatrix_.block<3, 3>(StateVx, StateAx) =
      Eigen::Matrix3d::Identity(3, 3) * deltaT;

  double sr = sin(x(StateRoll));
  double cr = cos(x(StateRoll));
  double tp = tan(x(StatePitch));
  double cp = cos(x(StatePitch));
  Eigen::Matrix3d T; // angular velocity -> angle transformation matrix
  T << 1, sr * tp, cr * tp, 0, cr, -sr, 0, sr / cp, cr / cp;
  transitionMatrix_.block<3, 3>(StateRoll, StateVroll) = T * deltaT;

  newState = transitionMatrix_ * x;
  if (output)
  {
    velocity << std::fixed << std::setprecision(3) << "last time:" << std::endl
             << Localization::getLastMeasurementTime() - startSec << std::endl;
    velocity << "deltaT = " << deltaT << std::endl
             << "x=" << std::endl
             << x << std::endl
             << "velocity is " << x(StateVz) << std::endl
             << "transition = " << std::endl
             << transitionMatrix_ << std::endl;

    velocity << "newstate = " << std::endl
             << newState << std::endl
             << std::endl;
  }
  return newState;
}

void Localization::setupProcessModel()
{
  filter_.setDifferenceFunction(
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &a,
         const Eigen::Matrix<double, STATE_SIZE, 1> &b)
          -> Eigen::Matrix<double, STATE_SIZE, 1>
      {
        Eigen::Matrix<double, STATE_SIZE, 1> diff = a - b;
        // wrap rpy
        // logfile<<"a is " << a(StateRoll)<<" "<<a(StatePitch)<<" "<<a(StateYaw)<< std::endl<<"b is "<<b(StateRoll)<<" "<<b(StatePitch)<<" "<<b(StateYaw)<<std::endl<<" (diff(StateRoll)):"<<diff(StateRoll) <<std::endl;

        diff(StateRoll) = NormalizeAngle(diff(StateRoll));
        diff(StatePitch) = NormalizeAngle(diff(StatePitch));
        diff(StateYaw) = NormalizeAngle(diff(StateYaw));
        // logfile<<" NormalizeAngle(diff(StateRoll)):"<<diff(StateRoll) <<std::endl;
        return diff;
      });

  filter_.setMeanFunction(
      [](const Eigen::Matrix<double, STATE_SIZE, 2 * STATE_SIZE + 1>
             &sigmaPoints,
         const Eigen::Matrix<double, 2 * STATE_SIZE + 1, 1> &weights)
          -> Eigen::Matrix<double, STATE_SIZE, 1>
      {
        Eigen::Matrix<double, STATE_SIZE, 1> mean;
        mean.setZero();
        // atan2(sum_sin, sum_cos) is used to wrap mean of angles
        double sum_sin[3] = {0, 0, 0};
        double sum_cos[3] = {0, 0, 0};
        for (int i = 0; i < sigmaPoints.cols(); i++)
        {
          for (int j = 0; j < STATE_SIZE; j++)
          {
            if (j == StateRoll)
            {
              // logfile<<"sigmaPoints(StateRoll, i)"<<sigmaPoints(StateRoll, i)<<std::endl;
              sum_sin[0] += sin(sigmaPoints(StateRoll, i)) * weights(i);
              sum_cos[0] += cos(sigmaPoints(StateRoll, i)) * weights(i);
            }
            else if (j == StatePitch)
            {
              sum_sin[1] += sin(sigmaPoints(StatePitch, i)) * weights(i);
              sum_cos[1] += cos(sigmaPoints(StatePitch, i)) * weights(i);
            }
            else if (j == StateYaw)
            {
              sum_sin[2] += sin(sigmaPoints(StateYaw, i)) * weights(i);
              sum_cos[2] += cos(sigmaPoints(StateYaw, i)) * weights(i);
            }
            else
            {
              mean(j) += sigmaPoints(j, i) * weights(i);
            }
          }
        }
        mean(StateRoll) = atan2(sum_sin[0], sum_cos[0]);
        mean(StatePitch) = atan2(sum_sin[1], sum_cos[1]);
        mean(StateYaw) = atan2(sum_sin[2], sum_cos[2]);
        // logfile<<"sum_sin is "<<sum_sin[0]<<" "<<sum_sin[1]<<" "<<sum_sin[2]<<std::endl<<"mean is "<<std::endl<<mean<<std::endl;
        return mean;
      });
}

void Localization::setupMeasurementModels()
{
  depthModel_.measurementFn =
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, DEPTH_SIZE, 1>
  {
    Eigen::Matrix<double, DEPTH_SIZE, 1> measurement;
    measurement(0) = state(StateZ);
    return measurement;
  };

  dvlModel_.measurementFn =
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, DVL_SIZE, 1>
  {
    Eigen::Matrix<double, DVL_SIZE, 1> measurement;
    measurement(0) = state(StateVx);
    measurement(1) = state(StateVy);
    measurement(2) = state(StateVz);
    return measurement;
  };

  imuModel_.measurementFn =
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, IMU_SIZE, 1>
  {
    Eigen::Matrix<double, IMU_SIZE, 1> measurement;
    measurement(ImuRoll) = state(StateRoll);
    measurement(ImuPitch) = state(StatePitch);
    measurement(ImuYaw) = state(StateYaw);
    measurement(ImuVroll) = state(StateVroll);
    measurement(ImuVpitch) = state(StateVpitch);
    measurement(ImuVyaw) = state(StateVyaw);
    measurement(ImuAx) = state(StateAx);
    measurement(ImuAy) = state(StateAy);
    measurement(ImuAz) = state(StateAz);
    return measurement;
  };
  // handle wrapping for imu's orienation values
  imuModel_.diffFn = [](const Eigen::Matrix<double, IMU_SIZE, 1> &a,
                        const Eigen::Matrix<double, IMU_SIZE, 1> &b)
      -> Eigen::Matrix<double, IMU_SIZE, 1>
  {
    Eigen::Matrix<double, IMU_SIZE, 1> diff = a - b;
    // wrap rpy
    // logfile<<std::fixed<<std::setprecision(4)<<"a(ImuRoll):"<<a(ImuRoll)<<" diff(ImuRoll):"<<diff(ImuRoll)<<std::endl;
    diff(ImuRoll) = NormalizeAngle(diff(ImuRoll));
    diff(ImuPitch) = NormalizeAngle(diff(ImuPitch));
    diff(ImuYaw) = NormalizeAngle(diff(ImuYaw));
    // logfile<<std::fixed<<std::setprecision(4)<<"NormalizeAngle(diff(ImuRoll) ):" <<diff(ImuRoll)<<std::endl;
    return diff;
  };
  imuModel_.meanFn =
      [](const Eigen::Matrix<double, IMU_SIZE, 2 * STATE_SIZE + 1> &sigmaPoints,
         const Eigen::Matrix<double, 2 * STATE_SIZE + 1, 1> &weights)
      -> Eigen::Matrix<double, IMU_SIZE, 1>
  {
    Eigen::Matrix<double, IMU_SIZE, 1> mean;
    mean.setZero();
    // atan2(sum_sin, sum_cos) is used to wrap mean of angles
    double sum_sin[3] = {0, 0, 0};
    double sum_cos[3] = {0, 0, 0};
    for (int i = 0; i < sigmaPoints.cols(); i++)
    {
      for (int j = 0; j < IMU_SIZE; j++)
      {
        if (j >= 0 and j < 3)
        { // index 0-2 is roll, pitch & yaw
          sum_sin[j] += sin(sigmaPoints(j, i)) * weights(i);
          sum_cos[j] += cos(sigmaPoints(j, i)) * weights(i);
        }
        else
        {
          mean(j) += sigmaPoints(j, i) * weights(i);
        }
      }
    }
    for (int i = 0; i < 3; i++)
    {
      mean(i) = atan2(sum_sin[i], sum_cos[i]);
    }
    return mean;
  };

  angleModel_.measurementFn =
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, ANGLE_SIZE, 1>
  {
    Eigen::Matrix<double, ANGLE_SIZE, 1> measurement;
    //beacon in world -> beacon in auv -> beacon in usbl -> angle

    Eigen::Matrix3d World2Base =
        (Eigen::AngleAxisd(state(StateYaw), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(state(StatePitch), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(state(StateRoll), Eigen::Vector3d::UnitX()))
            .toRotationMatrix(); // rotation matrix (ZYX)
    Eigen::Vector3d base2beaconInworld;
    base2beaconInworld << state(StateBeaconX) - state(StateX), state(StateBeaconY) - state(StateY), state(StateBeaconZ) - state(StateZ);
    Eigen::Vector3d base2beaconInbase = World2Base.transpose() * base2beaconInworld;
    Eigen::Matrix3d Base2USBL =
        (Eigen::AngleAxisd(state(StateUsblY), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(state(StateUsblP), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(state(StateUsblR), Eigen::Vector3d::UnitX()))       
        .toRotationMatrix(); // rotation matrix (ZYX)
    Eigen::Vector3d base2beaconInUSBL = Base2USBL.transpose() * base2beaconInbase;

    measurement(0) = atan2(base2beaconInUSBL.y(), base2beaconInUSBL.x());
    measurement(1) = asin(base2beaconInUSBL.z() / base2beaconInUSBL.norm());
    return measurement;
  };

  // handle wrapping for imu's orienation values
  angleModel_.diffFn = [](const Eigen::Matrix<double, ANGLE_SIZE, 1> &a,
                          const Eigen::Matrix<double, ANGLE_SIZE, 1> &b)
      -> Eigen::Matrix<double, ANGLE_SIZE, 1>
  {
    Eigen::Matrix<double, ANGLE_SIZE, 1> diff = a - b;
    diff(0) = NormalizeAngle(diff(0));
    diff(1) = NormalizeAngle(diff(1));
    return diff;
  };
  angleModel_.meanFn =
      [](const Eigen::Matrix<double, ANGLE_SIZE, 2 * STATE_SIZE + 1> &sigmaPoints,
         const Eigen::Matrix<double, 2 * STATE_SIZE + 1, 1> &weights)
      -> Eigen::Matrix<double, ANGLE_SIZE, 1>
  {
    Eigen::Matrix<double, ANGLE_SIZE, 1> mean;
    mean.setZero();
    // atan2(sum_sin, sum_cos) is used to wrap mean of angles
    double sum_sin[2] = {0, 0};
    double sum_cos[2] = {0, 0};
    for (int i = 0; i < sigmaPoints.cols(); i++)
    {
      for (int j = 0; j < ANGLE_SIZE; j++)
      {
        sum_sin[j] += sin(sigmaPoints(j, i)) * weights(i);
        sum_cos[j] += cos(sigmaPoints(j, i)) * weights(i);
      }
    }
    for (int i = 0; i < ANGLE_SIZE; i++)
    {
      mean(i) = atan2(sum_sin[i], sum_cos[i]);
    }
    return mean;
  };
  recvimModel_.measurementFn =
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, RECVIM_SIZE, 1>
  {
    Eigen::Matrix<double, RECVIM_SIZE, 1> measurement;
    //beacon in world -> beacon in auv -> beacon in usbl -> angle

    Eigen::Matrix3d World2Base =
        (Eigen::AngleAxisd(state(StateYaw), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(state(StatePitch), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(state(StateRoll), Eigen::Vector3d::UnitX()))
            .toRotationMatrix(); // rotation matrix (ZYX)
    Eigen::Vector3d base2beaconInworld;
    base2beaconInworld << state(StateBeaconX) - state(StateX), state(StateBeaconY) - state(StateY), state(StateBeaconZ) - state(StateZ);
    Eigen::Vector3d base2beaconInbase = World2Base.transpose() * base2beaconInworld;
    // Eigen::Matrix3d Base2USBL =
    //     Eigen::AngleAxisd(state(StateTheta), Eigen::Vector3d::UnitZ()).toRotationMatrix(); // rotation matrix (ZYX)
    // Eigen::Vector3d base2beaconInUSBL = Base2USBL.transpose() * base2beaconInbase;
    base2beaconInbase.normalize();
    Eigen::Vector3d velocity;
    velocity << state(StateVx), state(StateVy), state(StateVz);
    measurement(0) = base2beaconInbase.transpose() * velocity;
    measurement(1) = state(StateBeaconZ);
    return measurement;
  };

  usblInitialModel_.measurementFn = 
      [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
      -> Eigen::Matrix<double, USBL_INITIAL_SIZE, 1>
  {
    Eigen::Matrix<double, USBL_INITIAL_SIZE, 1> measurement;
    //beacon in world -> beacon in auv -> beacon in usbl -> angle
    measurement << state(StateBeaconX), state(StateBeaconY), state(StateBeaconZ), state(StateUsblR), state(StateUsblP), state(StateUsblY);
    return measurement;
  };

  // handle wrapping for imu's orienation values
  usblInitialModel_.diffFn = [](const Eigen::Matrix<double, USBL_INITIAL_SIZE, 1> &a,
                          const Eigen::Matrix<double, USBL_INITIAL_SIZE, 1> &b)
      -> Eigen::Matrix<double, USBL_INITIAL_SIZE, 1>
  {
    Eigen::Matrix<double, USBL_INITIAL_SIZE, 1> diff = a - b;
    for (int i = 3; i < 6; i ++)
    {
      diff(i) = NormalizeAngle(diff(i));
    }

    return diff;
  };
  usblInitialModel_.meanFn =
      [](const Eigen::Matrix<double, USBL_INITIAL_SIZE, 2 * STATE_SIZE + 1> &sigmaPoints,
         const Eigen::Matrix<double, 2 * STATE_SIZE + 1, 1> &weights)
      -> Eigen::Matrix<double, USBL_INITIAL_SIZE, 1>
  {
    Eigen::Matrix<double, USBL_INITIAL_SIZE, 1> mean;
    mean.setZero();
    // atan2(sum_sin, sum_cos) is used to wrap mean of angles
    double sum_sin[3] = {0, 0, 0};
    double sum_cos[3] = {0, 0, 0};
    for (int i = 0; i < sigmaPoints.cols(); i++)
    {
      for (int j = 0; j < USBL_INITIAL_SIZE; j++)
      {
        if (j >= 3)
        { // index 0-2 is roll, pitch & yaw
          sum_sin[j - 3] += sin(sigmaPoints(j, i)) * weights(i);
          sum_cos[j - 3] += cos(sigmaPoints(j, i)) * weights(i);
        }
        else
        {
          mean(j) += sigmaPoints(j, i) * weights(i);
        }
      }
    }

    for (int i = 0; i < 3; i++)
    {
      mean(i + 3) = atan2(sum_sin[i], sum_cos[i]);
    }
    return mean;
  };
}
//   recvimModel_.measurementFn =
//       [](const Eigen::Matrix<double, STATE_SIZE, 1> &state)
//       -> Eigen::Matrix<double, RECVIM_SIZE, 1>
//   {
//     Eigen::Matrix<double, RECVIM_SIZE, 1> measurement;
//     //beacon in world -> beacon in auv -> beacon in usbl -> angle

//     Eigen::Matrix3d World2Base =
//         (Eigen::AngleAxisd(state(StateYaw), Eigen::Vector3d::UnitZ()) *
//          Eigen::AngleAxisd(state(StatePitch), Eigen::Vector3d::UnitY()) *
//          Eigen::AngleAxisd(state(StateRoll), Eigen::Vector3d::UnitX()))
//             .toRotationMatrix(); // rotation matrix (ZYX)
//     Eigen::Vector3d base2beaconInworld;
//     base2beaconInworld << state(StateBeaconX) - state(StateX), state(StateBeaconY) - state(StateY), state(StateBeaconZ) - state(StateZ);
//     Eigen::Vector3d base2beaconInbase = World2Base.transpose() * base2beaconInworld;
//     Eigen::Matrix3d Base2USBL =
//         Eigen::AngleAxisd(state(StateTheta), Eigen::Vector3d::UnitZ()).toRotationMatrix(); // rotation matrix (ZYX)
//     Eigen::Vector3d base2beaconInUSBL = Base2USBL.transpose() * base2beaconInbase;
//     base2beaconInUSBL.normalize();
//     Eigen::Vector3d velocity;
//     velocity << state(StateVx), state(StateVy), state(StateVz);
//     measurement(0) = base2beaconInUSBL.transpose() * velocity;
//     measurement(1) = state(StateBeaconZ);
//     return measurement;
//   };
// }
void Localization::setInitialCovariance(
    const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &P)
{
  initialP_ = P;
  reset();
}

void Localization::setProcessNoise(
    const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &Q)
{
  filter_.setProcessNoise(Q);
  reset();
}

void Localization::setParam(bool INITIALIZATION_NLS, double threshold)
{
  INITIALIZATION_NLS_ = INITIALIZATION_NLS;
  initial_usbl.set_threshold(threshold);
  std::cout << "get threshold : "<<initial_usbl.get_threshold() << std::endl;
  std::cout << "Now localization set threshold is " << threshold << std::endl;
  reset();
  std::cout << "get threshold after reset : "<<initial_usbl.get_threshold() << std::endl << std::endl;

}
Eigen::Matrix<double, STATE_SIZE, 1> &Localization::getState()
{
  return filter_.getState();
}

Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &Localization::getCovariance()
{
  return filter_.getCovariance();
}

void Localization::setState(const Eigen::Matrix<double, STATE_SIZE, 1> &x)
{
  filter_.setState(x);
}

bool Localization::isInitialized() { return isInitialized_; }

double Localization::getLastMeasurementTime() { return lastMeasurementTime_; }

void Localization::setLastMeasurementTime(double lastime) { lastMeasurementTime_ = lastime; }
