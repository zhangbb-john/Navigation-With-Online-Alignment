#pragma once


#include <eigen3/Eigen/Dense>
#include <memory>
#include <queue>

//#include <au_core/loader_util.h>
#include <random>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "sensor_msgs/FluidPressure.h"
#include "auv_nav_msg/DVL.h"
#include "auv_nav_msg/USBLLONG.h"
#include "auv_nav_msg/USBLANGLES.h"
#include "auv_nav_msg/RECVIM.h"
#include "auv_nav_msg/State.h"
#include "cola2_msgs/AHRS.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "auv_nav_msg/GPSInfo.h"
#include "auv_nav_msg/State.h"

#include "cola2_msgs/GPS.h"
#include <localization.h>

// #define BAG
#define ENU
#define DOWN
#define ROTATE
#if METHOD == DOA
#define ADJUST_NOISE
#endif
#if EXPERIMENT == FIELD
#define IMU_MSG AHRS
#define FAKE_RECVIM
#define MAG_ATT
// #define USE_FIXED_ORIGIN
#endif
#if SITUATION == INFERENCE
#define ADD_NOISE 0.0
#define MISALIGN 0.20943951023931953//0.15707963267948966//0.1047 //0.05235987755982988//0.0//-0.05235987755982988//(-3) //-0.1047 //-0.15707963267948966 //-0.1047//-0.15707963267948966//-0.05235987755982988//(-3)//0.0 // 0.05235987755982988 //3    //0.15707963267948966 //9  //0.1047 //6
#else
#define ADD_NOISE 0.0
#define MISALIGN 0.0
#endif

#ifdef BAG
#include <rosgraph_msgs/Clock.h>
#endif

using MeasurementQueue =
    std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>,
                        Measurement>;

/*
 * ROS wrapper for Localization class.
 * subscribes to all sensor inputs
 * publishes filtered output message
 */
class LocalizationRos {
 public:
  // constructor
  explicit LocalizationRos(const ros::NodeHandle& nh,
                           const ros::NodeHandle& private_nh);

  // destructor - closes all subscriber connections and clears the message
  // filters
  ~LocalizationRos() = default;

  // resets filter to initial state
  void reset();

 protected:
  // periodically called by ros::Timer
  void update(const ros::TimerEvent& event);

  // callback method for receiving IMU messages
  #if IMU_MSG == AHRS
  void imuCallback(const cola2_msgs::AHRSConstPtr &msg);
  #elif IMU_MSG == SENSOR_MSG_IMU
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  #else
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  #endif
  // callback method for receiving DVL messages
  void dvlCallback(const auv_nav_msg::DVLConstPtr& msg);

  // callback method for recieving depth messages
  void depthCallback(const sensor_msgs::FluidPressureConstPtr& msg);
  // callback method for recieving usbl messages
  void usbllongCallback(const auv_nav_msg::USBLLONGConstPtr& msg);
  void usblanglesCallback(const auv_nav_msg::USBLANGLESConstPtr& msg);
  void usblrecvimCallback(const auv_nav_msg::RECVIMConstPtr& msg);
  void gpsCallback(const cola2_msgs::GPS msg);

#ifdef BAG
  void clockCallback(const rosgraph_msgs::ClockConstPtr& msg);
#endif

  // returns the ekf output as ros message for publishing
  nav_msgs::Odometry getFilteredOdomMessage();
  auv_nav_msg::State getStateMessage();
  void publishOdomTf();
  // gets transform frame from message from to baselink
  tf2::Transform getTransformFrame(const std_msgs::Header& header);

  // loads ukf params from file
  void loadParams();

  // loads square eigen matrix from params
  void loadMatrixFromParams(Eigen::MatrixXd& mat, const std::string& key);

  // clears measurement queue
  void clearMeasurementQueue();

  // rotates covariance array using quaternion q
  // result copies to Eigen::Matrix rotated
  void rotateCovariance(const double* covariance, const tf2::Quaternion& q,
                        Eigen::MatrixXd& rotated);

  // ros node handles
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  struct Params {
    // filter update rate
    double frequency;
    double dvl_noise_sigma;
    double depth_noise_sigma;
    double recvim_noise_sigma;
    double threshold;
    bool PRIOR;
    bool INITIALIZATION_NLS;

    // ukf sigma parameters
    Eigen::MatrixXd initialCov;
    Eigen::MatrixXd processNoiseCov;

    Params()
        : initialCov(STATE_SIZE, STATE_SIZE),
          processNoiseCov(STATE_SIZE, STATE_SIZE) {}
  } params_;

  // localization ukf filter
  Localization filter_;

  ros::Publisher odomPub_;
  ros::Publisher statePub_;
  ros::Publisher gpsInfoPub_;

  ros::Timer updateTimer_;
  // subscribers for sensors
  ros::Subscriber imuSub_;
  ros::Subscriber dvlSub_;
  ros::Subscriber depthSub_;
  ros::Subscriber usbllongSub_;
  ros::Subscriber usblanglesSub_;
  ros::Subscriber recvimSub_;
  ros::Subscriber gpsSub_;
#ifdef BAG
  ros::Subscriber clockSub_;
  double time_record;
#endif

  // frames ids
  std::string baseLinkFrame_;
  std::string odomFrame_;

  // tf buffer for managing coord frames
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster odomBroadcaster_;

  // queue measurements until update method is trigger
  MeasurementQueue measurementQueue_;
  double last_imu_time = 0;
  //Diverge judge
  double dist = 0;
  double diverge_start = 0;
  bool Diverge = false;
  bool ERR = false;

  // GPS
  bool is_gps_init = false;
  // double GPS_start_x = 0;
  // double GPS_start_y = 0;
  double gps_start_lat = 0;
  double gps_start_lon = 0;
  double last_GPS_time = 0;
  // double roll, pitch, yaw;
  double position_uncertainty = 1.0e6;
  double distance = 0, t_start = 0;
  Eigen::Vector3d last_pos;
  Eigen::Vector3d beaconPos;
  auv_nav_msg::State state_topic;

  int accept_num = 0, reject_num = 0;
  //Fixed Origin 
  double fixed_lat = 30.108557887083336; //18.318901930804600; //30.10929164584 
  double fixed_lon =  1.219918879456944e+02;//1.091455029943104e+02; //121.991848022267;
  double odom_vs_state_x = 0, odom_vs_state_y = 0;
  // IMU
  int imu_count = 0, ins_count = 0;
  double ins_coord_lat = 0, ins_coord_lon = 0, ins_start_x = 0, ins_start_y = 0;    //ins coordinate in radian
  bool is_ins_init = false;

  // USBL
  bool is_usbl_init = false;
  double min_bearing = 10, max_bearing = -10;
};
