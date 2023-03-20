
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/FluidPressure.h>
#include <auv_nav_msg/DVL.h>
#include <sensor_msgs/Imu.h>
#include <data_gen/motion.h>
#include <random>
#include "auv_nav_msg/USBLANGLES.h"
#include "auv_nav_msg/RECVIM.h"
#include <eigen3/Eigen/Core>
#include "auv_nav_msg/State.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define R
#define ENU
Eigen::Vector3d imu_gyro_bias;
Eigen::Vector3d imu_acc_bias;
std::ofstream log_file;

struct ImuData
{
    double timestamp;
    Eigen::Vector3d euler_angle;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};
struct DvlData
{
    double timestamp;
    Eigen::Vector3d vel;
    Eigen::Vector3d euler;
};
struct DepthData
{
    double timestamp;
    double depth;
};
struct AngleData
{
    double timestamp;
    double bearing;
    double elevation;
};
struct RecvimData
{
    double timestamp;
    double speed;
    double depth;
};
ImuData imu_gen(MotionData motion, double t)
{
    ImuData data;

    //acc
    Eigen::Vector3d gn(0, 0, -9.81); //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    //data.acc  = motion.Rwb.transpose() * ( motion.acc -  gn );  //  Rbw * Rwn * gn = gs
    data.acc = motion.Rwb.transpose() * (motion.acc + gn);

    //gyro
    data.gyro = eulerRates2bodyRates(motion.eulerAngles) * motion.eulerAnglesRates; //  euler rates trans to body gyro

    // euler_angle
    data.euler_angle = motion.eulerAngles;
    data.timestamp = t;
    return data;
}
void addIMUnoise(ImuData &data, IMUParam imu_param)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);
    // double gyro_noise_sigma = 1e-3, gyro_bias_sigma = 5e-5, acc_noise_sigma = 1e-2, acc_bias_sigma = 5e-5, roll_sigma = 0.02, pitch_sigma = 0.02, yaw_sigma = 0.04; //0.5;

    Eigen::Vector3d noise_gyro(noise(generator_), noise(generator_), noise(generator_));
    // std::cout << "noise_gyro:" << noise_gyro << std::endl;
    Eigen::Matrix3d gyro_sqrt_cov = imu_param.gyro_noise_sigma * Eigen::Matrix3d::Identity();
    data.gyro = data.gyro + gyro_sqrt_cov * noise_gyro + imu_gyro_bias;

    Eigen::Vector3d noise_acc(noise(generator_), noise(generator_), noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = imu_param.acc_noise_sigma * Eigen::Matrix3d::Identity();
    data.acc = data.acc + acc_sqrt_cov * noise_acc + imu_acc_bias;
    Eigen::Vector3d noise_vector = acc_sqrt_cov * noise_acc + imu_acc_bias;
    log_file << noise_vector.x() << std::endl;
    Eigen::Vector3d noise_angle(noise(generator_), noise(generator_), noise(generator_));
    Eigen::Matrix3d angle_sqrt_cov;
    angle_sqrt_cov << imu_param.roll_sigma, 0, 0,
        0, imu_param.pitch_sigma, 0,
        0, 0, imu_param.yaw_sigma;
    data.euler_angle = data.euler_angle + angle_sqrt_cov * noise_angle;

    //gyro_bias update
    Eigen::Vector3d noise_gyro_bias(noise(generator_), noise(generator_), noise(generator_));
    imu_gyro_bias += imu_param.gyro_bias_sigma * noise_gyro_bias;

    //acc_bias update
    Eigen::Vector3d noise_acc_bias(noise(generator_), noise(generator_), noise(generator_));
    imu_acc_bias += imu_param.acc_bias_sigma * noise_acc_bias;
    // // gyro_bias update
    // Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
    // gyro_bias_ += gyro_bias_sigma * sqrt(imu_timestep ) * noise_gyro_bias;
    // data.imu_gyro_bias = gyro_bias_;

    // // acc_bias update
    // Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
    // acc_bias_ += acc_bias_sigma * sqrt(imu_timestep ) * noise_acc_bias;
    // data.imu_acc_bias = acc_bias_;
}
DvlData dvl_gen(MotionData motion, double t)
{
    DvlData data;
    // Eigen::Vector3d euler(M_PI /2  , M_PI/3 , M_PI/4);
    //Eigen::Vector3d euler(0  , 0 , M_PI/6);
    Eigen::Matrix3d IMU2DVL;
#ifdef R
    IMU2DVL << 0.717219, 0.696831, -0.004789,
        -0.696764, 0.717223, 0.010524,
        0.010768, -0.004211, 0.999933;
#else
    IMU2DVL << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
#endif
    Eigen::Matrix3d W2DVL = motion.Rwb * IMU2DVL;
    data.vel = W2DVL.transpose() * motion.velocity;
    data.euler = W2DVL.eulerAngles(2, 1, 0);
    data.timestamp = t;
    return data;
}

void addDVLnoise(DvlData &data, double dvl_noise_sigma, double dvl_scale)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    Eigen::Vector3d noise_dvl(noise(generator_), noise(generator_), noise(generator_));
    Eigen::Matrix3d dvl_sqrt_cov = dvl_noise_sigma * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d dvl_scale_mat = dvl_scale * Eigen::Matrix3d::Identity();
    data.vel = dvl_scale_mat * (data.vel + dvl_sqrt_cov * noise_dvl);
}

DepthData depth_gen(MotionData motion, double t)
{
    DepthData data;
    data.depth = motion.twb(2);
    data.timestamp = t;
    return data;
}

void addDEPTHnoise(DepthData &data, double depth_noise_sigma)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);
    data.depth = data.depth + depth_noise_sigma * noise(generator_);
}

AngleData angle_gen(MotionData motion, double t, Eigen::Vector3d usbl_rpy, Eigen::Vector3d beacon_pos)
{
    AngleData data;
    Eigen::Vector3d base2beaconInworld = beacon_pos - motion.twb;
    Eigen::Vector3d base2beaconInbase = motion.Rwb.transpose() * base2beaconInworld;

    Eigen::Matrix3d Base2USBL =
        (Eigen::AngleAxisd(usbl_rpy[2], Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(usbl_rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(usbl_rpy[0], Eigen::Vector3d::UnitX()))       
        .toRotationMatrix();        
    Eigen::Vector3d base2beaconInUSBL = Base2USBL.transpose() * base2beaconInbase;

    data.bearing = atan2(base2beaconInUSBL.y(), base2beaconInUSBL.x());
    data.elevation = asin(base2beaconInUSBL.z() / base2beaconInUSBL.norm());
    // std::cout <<"base2beaconInUSBL.z() is " << base2beaconInUSBL.z() << ";base2beaconInUSBL.norm() is " << base2beaconInUSBL.norm() << "; elevation is " << data.elevation << std::endl;
    data.timestamp = t;
    return data;
}

void addANGLEnoise(AngleData &data, double angle_noise_sigma)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);
    data.bearing = data.bearing + angle_noise_sigma * noise(generator_);
    data.elevation = data.elevation + angle_noise_sigma * noise(generator_);
}

RecvimData recvim_gen(MotionData motion, double t, Eigen::Vector3d beacon_pos)
{
    RecvimData data;
    Eigen::Vector3d base2beaconInworld = beacon_pos - motion.twb;
    Eigen::Vector3d base2beaconInbase = motion.Rwb.transpose() * base2beaconInworld;
    base2beaconInbase.normalize();
    Eigen::Vector3d velocity = motion.Rwb.transpose() * motion.velocity;
    data.speed = base2beaconInbase.transpose() * velocity;
    data.timestamp = t;
    data.depth = beacon_pos.z();
    return data;
}

void addRECVIMnoise(RecvimData &data, double recvim_noise_sigma, double beacon_depth_noise_sigma)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);
    data.speed = data.speed + recvim_noise_sigma * noise(generator_);
    data.depth = data.depth + beacon_depth_noise_sigma * noise(generator_);
}
nav_msgs::Odometry odom_gen(MotionData motion, double t)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time().fromSec(t);
    odom.header.frame_id = "temp_odom";
    odom.pose.pose.position.x = motion.twb(0);
    odom.pose.pose.position.y = motion.twb(1);
    odom.pose.pose.position.z = motion.twb(2);
    Eigen::Quaternionf q= Eigen::AngleAxisf(motion.eulerAngles(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(motion.eulerAngles(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(motion.eulerAngles(0), Eigen::Vector3f::UnitX());
    odom.pose.pose.orientation.x = q.coeffs().x();
    odom.pose.pose.orientation.y = q.coeffs().y();
    odom.pose.pose.orientation.z = q.coeffs().z();
    odom.pose.pose.orientation.w = q.coeffs().w();
    Eigen::Vector3d velocity = motion.Rwb.transpose() * motion.velocity;
    odom.twist.twist.linear.x = velocity.x();
    odom.twist.twist.linear.y = velocity.y();
    odom.twist.twist.linear.z = velocity.z();   
    Eigen::Vector3d gyro = eulerRates2bodyRates(motion.eulerAngles) * motion.eulerAnglesRates; //  euler rates trans to body gyro
    odom.twist.twist.angular.x = gyro(0);
    odom.twist.twist.angular.y = gyro(1);
    odom.twist.twist.angular.z = gyro(2);
    return odom;
                //     imu.orientation.x = q.coeffs().x();
                // imu.orientation.y = q.coeffs().y();
                // imu.orientation.z = q.coeffs().z();
                // imu.orientation.w = q.coeffs().w();
}
auv_nav_msg::State state_gen(MotionData motion, double t, Eigen::Vector3d usbl_rpy, Eigen::Vector3d beacon_pos)
{
    auv_nav_msg::State state;
    state.header.stamp = ros::Time().fromSec(t);
    state.position.x = motion.twb(0);
    state.position.y = motion.twb(1);
    state.position.z = motion.twb(2);
    Eigen::Quaternionf q= Eigen::AngleAxisf(motion.eulerAngles(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(motion.eulerAngles(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(motion.eulerAngles(0), Eigen::Vector3f::UnitX());

    tf2::Quaternion quat(q.x(), q.y(), q.z(), q.w());
    tf2::Matrix3x3 orientation(quat);
    double roll, pitch, yaw;
    orientation.getRPY(roll, pitch, yaw);

    state.rpy.x = roll;
    state.rpy.y = pitch;
    state.rpy.z = yaw;

    Eigen::Vector3d v = motion.Rwb.transpose() * motion.velocity;

    state.velocity.x = v.x();
    state.velocity.y = v.y();
    state.velocity.z = v.z();
    Eigen::Vector3d gn(0, 0, -9.81); //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    //data.acc  = motion.Rwb.transpose() * ( motion.acc -  gn );  //  Rbw * Rwn * gn = gs
    Eigen::Vector3d acc = motion.Rwb.transpose() * (motion.acc);

    state.acceleration.x = acc.x();
    state.acceleration.y = acc.y();
    state.acceleration.z = acc.z();

    Eigen::Vector3d gyro = eulerRates2bodyRates(motion.eulerAngles) * motion.eulerAnglesRates; //  euler rates trans to body gyro

    state.angular_velocity.x = gyro.x();
    state.angular_velocity.y = gyro.y();
    state.angular_velocity.z = gyro.z();
    state.beacon_pos.x = beacon_pos.x();
    state.beacon_pos.y = beacon_pos.y();
    state.beacon_pos.z = beacon_pos.z();
    state.usbl_rpy.x = usbl_rpy[0];
    state.usbl_rpy.y = usbl_rpy[1];
    state.usbl_rpy.z = usbl_rpy[2];
    // state.position_covariance_diag.x = 1;
    // state.position_covariance_diag.y = covarianceMatrix(StateY, StateY);
    // state.position_covariance_diag.z = covarianceMatrix(StateZ, StateZ);
    // state.rpy_covariance_diag.x = covarianceMatrix(StateRoll, StateRoll);
    // state.rpy_covariance_diag.y = covarianceMatrix(StatePitch, StatePitch);
    // state.rpy_covariance_diag.z = covarianceMatrix(StateYaw, StateYaw);
    // state.velocity_covariance_diag.x = covarianceMatrix(StateVx, StateVx);
    // state.velocity_covariance_diag.y = covarianceMatrix(StateVy, StateVy);
    // state.velocity_covariance_diag.z = covarianceMatrix(StateVz, StateVz);
    // state.acceleration_covariance_diag.x = covarianceMatrix(StateAx, StateAx);
    // state.acceleration_covariance_diag.y = covarianceMatrix(StateAy, StateAy);
    // state.acceleration_covariance_diag.z = covarianceMatrix(StateAz, StateAz);
    // state.angular_velocity_covariance_diag.x = covarianceMatrix(StateVroll, StateVroll);
    // state.angular_velocity_covariance_diag.y = covarianceMatrix(StateVpitch, StateVpitch);
    // state.angular_velocity_covariance_diag.z = covarianceMatrix(StateVyaw, StateVyaw);
    // state.beacon_pos_covariance_diag.x = covarianceMatrix(StateBeaconX, StateBeaconX);
    // state.beacon_pos_covariance_diag.y = covarianceMatrix(StateBeaconY, StateBeaconY);
    // state.beacon_pos_covariance_diag.z = covarianceMatrix(StateBeaconZ, StateBeaconZ);
    // state.usbl_rpy_covariance.x = covarianceMatrix(StateUsblR, StateUsblR);
    // state.usbl_rpy_covariance.y = covarianceMatrix(StateUsblP, StateUsblP);
    // state.usbl_rpy_covariance.z = covarianceMatrix(StateUsblY, StateUsblY);
    return state;

}
void print_info(Param params)
{
    std::cout << "The parameters are as follows:" << std::endl;
    std::cout << "trajectory type is " << params.motion_param.type << std::endl;
    std::cout << "t_start is " << params.motion_param.t_start << std::endl;
    std::cout << "t_end is " << params.motion_param.t_end << std::endl;
    std::cout << "circle_num is " << params.motion_param.circle_num << std::endl;
    std::cout << "x_mag is " << params.motion_param.x_mag << std::endl;
    std::cout << "y_mag is " << params.motion_param.y_mag << std::endl;
    std::cout << "z_mag is " << params.motion_param.z_mag << std::endl;
    std::cout << "rp_mag is " << params.motion_param.rp_mag << std::endl;
    std::cout << "yaw_mag is " << params.motion_param.yaw_mag << std::endl;
    std::cout << "augular_speed is " << params.motion_param.augular_speed << std::endl;

    std::cout << "Failure: " << params.failure.Failure << std::endl;
    std::cout << "failure_t_start: " << params.failure.failure_t_start << std::endl;
    std::cout << "failure_t_end: " << params.failure.failure_t_end << std::endl;

    std::cout << "imu_timestep:" << params.measurement_param.imu_timestep << std::endl;
    std::cout << "dvl_timestep:" << params.measurement_param.dvl_timestep << std::endl;
    std::cout << "depth_timestep" << params.measurement_param.depth_timestep << std::endl;
    std::cout << "angle_timestep" << params.measurement_param.angle_timestep << std::endl;
    std::cout << "recvim_timestep" << params.measurement_param.recvim_timestep << std::endl;

    std::cout << "roll_sigma: " << params.imu_param.roll_sigma << std::endl;
    std::cout << "pitch_sigma: " << params.imu_param.pitch_sigma << std::endl;
    std::cout << "yaw_sigma: " << params.imu_param.yaw_sigma << std::endl;
    std::cout << "gyro_bias_sigma: " << params.imu_param.gyro_bias_sigma << std::endl;
    std::cout << "acc_bias_sigma: " << params.imu_param.acc_bias_sigma << std::endl;
    std::cout << "gyro_noise_sigma: " << params.imu_param.gyro_noise_sigma << std::endl;
    std::cout << "acc_noise_sigma: " << params.imu_param.acc_noise_sigma << std::endl;
    std::cout << "dvl_noise_sigma: " << params.dvl_noise_sigma << std::endl;
    std::cout << "dvl_scale: " << params.dvl_scale << std::endl;

    std::cout << "depth_noise_sigma: " << params.depth_noise_sigma << std::endl;
    std::cout << "angle_noise_sigma: " << params.angle_noise_sigma << std::endl;
    std::cout << "recvim_noise_sigma: " << params.recvim_noise_sigma << std::endl;
    std::cout << "beacon_depth_noise_sigma: " << params.beacon_depth_noise_sigma << std::endl;
    std::cout << "usbl_rpy: " << params.usbl_rpy.transpose() << std::endl;

    std::cout << "beacon_pos: " << params.beacon_pos.transpose() << std::endl;
    std::cout << "USBL: " << params.USBL << std::endl;
    std::cout << "NOISE: " << params.NOISE << std::endl;
    std::cout << "DEPTH: " << params.DEPTH << std::endl;
}

int main(int argc, char **argv)
{
    std::cout << "pub_node: 21/06/24/21:42" << std::endl;
    ROS_INFO("pub_node: 21/06/24/21:42 ");
    ros::init(argc, argv, "pub_node");
    ros::NodeHandle pub_node;
    ros::NodeHandle privateNh("~");

    // ros::Publisher imu_pub = pub_node.advertise<sensor_msgs::Imu>("/Sensor/AHRS",1, true);
    ros::Publisher imu_pub = pub_node.advertise<sensor_msgs::Imu>("/Sensor/AHRS", 1, true);

    ros::Publisher dvl_pub = pub_node.advertise<auv_nav_msg::DVL>("/Sensor/DVL", 1, true);

    ros::Publisher depth_pub = pub_node.advertise<sensor_msgs::FluidPressure>("/Sensor/Pressure", 1, true);
    ros::Publisher angle_pub = pub_node.advertise<auv_nav_msg::USBLANGLES>("/Sensor/USBLANGLES", 1, true);
    ros::Publisher recvim_pub = pub_node.advertise<auv_nav_msg::RECVIM>("/Sensor/RECVIM", 1, true);
    ros::Publisher truth_pub = pub_node.advertise<nav_msgs::Odometry>("/Locater/Truth", 1, true);
    ros::Publisher state_pub = pub_node.advertise<auv_nav_msg::State>("/TrueState", 20);
    sensor_msgs::Imu imu;
    auv_nav_msg::DVL dvl;
    sensor_msgs::FluidPressure depth;
    auv_nav_msg::USBLANGLES usblangle;
    auv_nav_msg::RECVIM recvim;
    // geometry_msgs::PoseStamped pose_stamped;
    nav_msgs::Odometry odometry_truth;
    Param params(privateNh);

    MOTION motionGen(params.motion_param);

    print_info(params);
    MotionData motion;
    Eigen::Quaternionf q;

    uint count = 0;
    double sec;
    uint period_imu = (uint)(params.measurement_param.imu_timestep * 100);
    //std::cout<<"period imu is "<<period_imu<<std::endl;
    uint period_dvl = (uint)(params.measurement_param.dvl_timestep * 100);
    uint period_depth = (uint)(params.measurement_param.depth_timestep * 100);
    uint period_angle = (uint)(params.measurement_param.angle_timestep * 100);
    uint period_recvim = (uint)(params.measurement_param.recvim_timestep * 100);
    std::string log_dir = "/home/ubuntu/Bearing-only/log";
    std::string dvl_str = log_dir + "/dvl2.txt";
    std::ofstream dvlfile2;
    dvlfile2.open(dvl_str.c_str(), std::ios::out);

    std::string rot_str = log_dir + "/rot2.txt";
    std::ofstream rotfile2;
    rotfile2.open(rot_str.c_str(), std::ios::out);

    std::string log_str = log_dir + "/log.txt";

    log_file.open(log_str.c_str(), std::ios::out);
    double startSec = 0;
    bool started = false;
    imu_acc_bias = Eigen::Vector3d::Zero();
    imu_gyro_bias = Eigen::Vector3d::Zero();
    while (count <= params.motion_param.t_end * 100)
    {
        usleep(10000);
        count++;
        if (count % period_imu == 0)
        {
            if (started == false)
            {
                startSec = ros::Time::now().toSec();
                started = true;
            }
            if (started == true)
            {   
                double nowSec = ros::Time::now().toSec();
                sec = nowSec- startSec;
                imu.header.stamp = ros::Time::now();
                motion = motionGen.MotionModel(sec);
                ImuData data = imu_gen(motion, sec);
                if (params.NOISE)
                    addIMUnoise(data, params.imu_param);
#ifdef ENU
                q = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(data.euler_angle(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(data.euler_angle(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(data.euler_angle(0), Eigen::Vector3f::UnitX());

#else
                q = Eigen::AngleAxisf(data.euler_angle(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(data.euler_angle(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(data.euler_angle(0), Eigen::Vector3f::UnitX());
#endif
                // q = Eigen::AngleAxisf(data.euler_angle(0), Eigen::Vector3f::UnitX())
                //     * Eigen::AngleAxisf(data.euler_angle(1), Eigen::Vector3f::UnitY())
                //     * Eigen::AngleAxisf(data.euler_angle(2), Eigen::Vector3f::UnitZ());
                //rotfile2 << std::fixed<<std::setprecision(5)<<"published rotation: "<<sec<<std::endl<<q.toRotationMatrix()<<std::endl;
                rotfile2 << std::fixed << std::setprecision(5) << sec << " data.euler_angle: " << data.euler_angle(0) << " " << data.euler_angle(1) << " " << data.euler_angle(2) << "  q:" << q.coeffs() << std::endl;
                rotfile2 << "Is it the same angle ? q.toRotationMatrix().eulerAngles(2,1,0): " << q.toRotationMatrix().eulerAngles(2, 1, 0).transpose()(2) << " " << q.toRotationMatrix().eulerAngles(2, 1, 0).transpose()(1) << " " << q.toRotationMatrix().eulerAngles(2, 1, 0).transpose()(0) << std::endl
                         << std::endl;

                imu.header.frame_id = "base_link";
                imu.orientation_covariance = {pow(params.imu_param.roll_sigma, 2), 0, 0, 0, pow(params.imu_param.pitch_sigma, 2), 0, 0, 0, pow(params.imu_param.yaw_sigma, 2)};
                // imu.orientation_covariance = {6e-7,0,0,0,6e-7,0,0,0,4e-6};
                // imu.orientation_covariance = {1e-4,0,0,0,1e-4,0,0,0,1e-4};//wrong param

                imu.linear_acceleration_covariance = {pow(params.imu_param.acc_noise_sigma, 2) * 1e2, 0, 0, 0, pow(params.imu_param.acc_noise_sigma, 2) * 1e2, 0, 0, 0, pow(params.imu_param.acc_noise_sigma, 2) * 1e2};
                imu.angular_velocity_covariance = {pow(params.imu_param.gyro_noise_sigma, 2), 0, 0, 0, pow(params.imu_param.gyro_noise_sigma, 2), 0, 0, 0, pow(params.imu_param.gyro_noise_sigma, 2)};

                //orientation
                // imu.orientation.x = -q.coeffs().x();
                // imu.orientation.y = -q.coeffs().y();
                // imu.orientation.z = -q.coeffs().z();
                // imu.orientation.w = q.coeffs().w();

                imu.orientation.x = q.coeffs().x();
                imu.orientation.y = q.coeffs().y();
                imu.orientation.z = q.coeffs().z();
                imu.orientation.w = q.coeffs().w();
                if (isnan(imu.orientation.y))
                {
                    std::cout << q.coeffs() << std::endl;
                    std::cout << "++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                    std::cout << "angle" << data.euler_angle(0) << std::endl;
                }
                //acc
                imu.linear_acceleration.x = data.acc(0);
                imu.linear_acceleration.y = data.acc(1);
                imu.linear_acceleration.z = data.acc(2);
                //v angular
                imu.angular_velocity.x = data.gyro(0);
                imu.angular_velocity.y = data.gyro(1);
                imu.angular_velocity.z = data.gyro(2);

                imu_pub.publish(imu);
                //ROS_INFO("imu published \n stamp: %f",sec);
                //ROS_INFO("[ x: %f y: %f z: %f]\n", imu.linear_acceleration.x,imu.orientation.y ,imu.orientation.z );
                //std::cout<<"published:"<<pose_stamped.pose.orientation.w<<" "<<pose_stamped.pose.orientation.x<<" "<<pose_stamped.pose.orientation.y<<" "<<pose_stamped.pose.orientation.z<<std::endl;
                truth_pub.publish(odom_gen(motion, nowSec));
                state_pub.publish(state_gen(motion, nowSec, params.usbl_rpy, params.beacon_pos));

            }
        }
        if (count % period_dvl == 0)
        {
            if (started == false)
            {
                startSec = ros::Time::now().toSec();
                started = true;
            }
            if (started == true)
            {
                double nowSec = ros::Time::now().toSec();
                sec = nowSec- startSec;
                if (params.failure.Failure && sec > params.failure.failure_t_start && sec < params.failure.failure_t_end)
                    std::cout << "DVL fails now" << std::endl;
                else
                {
                    motion = motionGen.MotionModel(sec);

                    DvlData data = dvl_gen(motion, sec);
                    if (params.NOISE)
                        addDVLnoise(data, params.dvl_noise_sigma, params.dvl_scale);

                    dvl.header.stamp = ros::Time::now();
                    dvl.header.frame_id = "base_link";
                    // dvl.velocity_covariance = {4e-4, 0, 0,
                    //                            0, 4e-4, 0,
                    //                            0, 0, 4e-4};
                    dvl.velocity_covariance = {pow(params.dvl_noise_sigma, 2), 0, 0,
                                               0, pow(params.dvl_noise_sigma, 2), 0,
                                               0, 0, pow(params.dvl_noise_sigma, 2)};
                    // dvl.velocity_covariance = {25 * 1e-4,0,0,
                    //                         0,25 * 1e-4,0,
                    //                         0,0,25 * 1e-4};
                    dvl.velocity_body.x = data.vel(0) * 1000;
                    dvl.velocity_body.y = data.vel(1) * 1000;
                    dvl.velocity_body.z = data.vel(2) * 1000;
                    dvl.h = data.euler(0) * 180 / M_PI;
                    dvl.p = data.euler(1) * 180 / M_PI;
                    dvl.r = data.euler(2) * 180 / M_PI + 180;
                    dvl_pub.publish(dvl);
                }
                //ROS_INFO("dvl published \n stamp: %f",sec);
                // pose_stamped.header.stamp = dvl.header.stamp;
                // pose_stamped.header.frame_id = dvl.header.frame_id;
                // pose_stamped.pose.position.x = motion.twb(0);
                // pose_stamped.pose.position.y = motion.twb(1);
                // pose_stamped.pose.position.z = motion.twb(2);
                // q = Eigen::AngleAxisf(motion.eulerAngles(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(motion.eulerAngles(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(motion.eulerAngles(0), Eigen::Vector3f::UnitX());
                // pose_stamped.pose.orientation.x = q.coeffs().x();
                // pose_stamped.pose.orientation.y = q.coeffs().y();
                // pose_stamped.pose.orientation.z = q.coeffs().z();
                // pose_stamped.pose.orientation.w = q.coeffs().w();
                // std::cout<<"dvl published:"<<pose_stamped.pose.orientation.w<<" "<<pose_stamped.pose.orientation.x<<" "<<pose_stamped.pose.orientation.y<<" "<<pose_stamped.pose.orientation.z<<std::endl;

                // dvlfile2 << std::fixed << std::setprecision(3) << "published velocity:"
                //          << " "
                //          << "time: " << sec << " " << motion.velocity.transpose() << " dvl: " << dvl.velocity_body.x << " " << dvl.velocity_body.y << " " << dvl.velocity_body.z << std::endl;

                //dvlfile<<std::fixed<<std::setprecision(3) << "published velocity:" <<std::endl << "time: " <<sec<<std::endl << data.vel << std::endl;
                //std::cout<<"published"<<std::endl;
                truth_pub.publish(odom_gen(motion, nowSec));
            }
        }
        if (count % period_depth == 0)
        {
            if (started == false)
            {
                startSec = ros::Time::now().toSec();
                started = true;
            }
            if (started == true)
            {
                double nowSec = ros::Time::now().toSec();
                sec = nowSec- startSec;
                motion = motionGen.MotionModel(sec);

                DepthData data = depth_gen(motion, sec);
                if (params.NOISE)
                    addDEPTHnoise(data, params.depth_noise_sigma);

                depth.fluid_pressure = data.depth;
                depth.header.stamp = ros::Time::now();
                depth.header.frame_id = "base_link";
                depth.variance = pow(params.depth_noise_sigma, 2);
                // depth.variance = 1e-4;

                depth_pub.publish(depth);
                //ROS_INFO("depth published \n stamp: %f",sec);

                // pose_stamped.header.stamp = depth.header.stamp;
                // pose_stamped.header.frame_id = depth.header.frame_id;
                // pose_stamped.pose.position.x = motion.twb(0);
                // pose_stamped.pose.position.y = motion.twb(1);
                // pose_stamped.pose.position.z = motion.twb(2);
                // q = Eigen::AngleAxisf(motion.eulerAngles(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(motion.eulerAngles(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(motion.eulerAngles(0), Eigen::Vector3f::UnitX());
                // pose_stamped.pose.orientation.x = q.coeffs().x();
                // pose_stamped.pose.orientation.y = q.coeffs().y();
                // pose_stamped.pose.orientation.z = q.coeffs().z();
                // pose_stamped.pose.orientation.w = q.coeffs().w();
                // truth_pub.publish(pose_stamped);
                truth_pub.publish(odom_gen(motion, nowSec));

                // std::cout<<"depth published:"<<pose_stamped.pose.orientation.w<<" "<<pose_stamped.pose.orientation.x<<" "<<pose_stamped.pose.orientation.y<<" "<<pose_stamped.pose.orientation.z<<std::endl;

                //std::cout<<"published"<<std::endl;
            }
        }
        if (count % period_angle == 0)
        {
            if (started == false)
            {
                startSec = ros::Time::now().toSec();
                started = true;
            }
            if (started == true)
            {
                double nowSec = ros::Time::now().toSec();
                sec = nowSec- startSec;                
                motion = motionGen.MotionModel(sec);

                AngleData data = angle_gen(motion, sec, params.usbl_rpy, params.beacon_pos);
                if (params.NOISE)
                    addANGLEnoise(data, params.angle_noise_sigma);

                usblangle.lbearing = data.bearing;
                usblangle.lelevation = data.elevation;
                usblangle.header.stamp = ros::Time::now();
                usblangle.header.frame_id = "base_link";
                usblangle.accuracy = params.angle_noise_sigma;
                if (params.USBL)
                    angle_pub.publish(usblangle);

                // pose_stamped.header.stamp = usblangle.header.stamp;
                // pose_stamped.header.frame_id = usblangle.header.frame_id;
                // pose_stamped.pose.position.x = motion.twb(0);
                // pose_stamped.pose.position.y = motion.twb(1);
                // pose_stamped.pose.position.z = motion.twb(2);
                // q = Eigen::AngleAxisf(motion.eulerAngles(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(motion.eulerAngles(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(motion.eulerAngles(0), Eigen::Vector3f::UnitX());
                // pose_stamped.pose.orientation.x = q.coeffs().x();
                // pose_stamped.pose.orientation.y = q.coeffs().y();
                // pose_stamped.pose.orientation.z = q.coeffs().z();
                // pose_stamped.pose.orientation.w = q.coeffs().w();
                // truth_pub.publish(pose_stamped);
                truth_pub.publish(odom_gen(motion, nowSec));

            }
        }
        if (count % period_recvim == 0)
        {
            if (started == false)
            {
                startSec = ros::Time::now().toSec();
                started = true;
            }
            if (started == true)
            {
                double nowSec = ros::Time::now().toSec();
                sec = nowSec- startSec;                
                motion = motionGen.MotionModel(sec);

                RecvimData data = recvim_gen(motion, sec, params.beacon_pos);
                if (params.NOISE)
                    addRECVIMnoise(data, params.recvim_noise_sigma, params.beacon_depth_noise_sigma);

                recvim.velocity = data.speed;
                recvim.header.stamp = ros::Time::now();
                recvim.header.frame_id = "base_link";
                std ::ostringstream str;
                str << std::fixed << std::setprecision(2) << "DEPTH," << data.depth;
                if (params.DEPTH)
                    recvim.data = str.str();
                if (params.USBL)
                    recvim_pub.publish(recvim);

                // pose_stamped.header.stamp = recvim.header.stamp;
                // pose_stamped.header.frame_id = recvim.header.frame_id;
                // pose_stamped.pose.position.x = motion.twb(0);
                // pose_stamped.pose.position.y = motion.twb(1);
                // pose_stamped.pose.position.z = motion.twb(2);
                // q = Eigen::AngleAxisf(motion.eulerAngles(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(motion.eulerAngles(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(motion.eulerAngles(0), Eigen::Vector3f::UnitX());
                // pose_stamped.pose.orientation.x = q.coeffs().x();
                // pose_stamped.pose.orientation.y = q.coeffs().y();
                // pose_stamped.pose.orientation.z = q.coeffs().z();
                // pose_stamped.pose.orientation.w = q.coeffs().w();
                // truth_pub.publish(pose_stamped);
                truth_pub.publish(odom_gen(motion, nowSec));

            }
        }
    }
    std::cout << "end" << std::endl;
    return 0;
}
