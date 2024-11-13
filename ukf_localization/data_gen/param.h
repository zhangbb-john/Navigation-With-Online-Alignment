#ifndef IMUSIM_PARAM_H
#define IMUSIM_PARAM_H
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
struct IMUParam{
    double roll_sigma;
    double pitch_sigma;
    double yaw_sigma;
    double gyro_bias_sigma;
    double acc_bias_sigma;

    double gyro_noise_sigma; // rad/s * 1/sqrt(hz)
    double acc_noise_sigma;  //　m/(s^2) * 1/sqrt(hz)
};
struct MeasurementParam{
    // time
    double imu_timestep;
    double dvl_timestep;
    double depth_timestep;
    double angle_timestep;
    double recvim_timestep;
};
struct MotionParam{
    int type;
    double t_start;
    double t_end; //  20 s
    float circle_num;
    //range
    float x_mag;
    float y_mag;
    float z_mag;
    //angle
    float yaw_mag;
    float rp_mag;
    float augular_speed;
};
struct Failure{
    bool Failure;
    double failure_t_start;
    double failure_t_end;
};
class Param
{
public:
    explicit Param(const ros::NodeHandle &private_nh);
    void print_test();
    void loadParams();

    MotionParam motion_param;
    Failure failure;
    MeasurementParam measurement_param;
    // noise
    IMUParam imu_param;
    double dvl_noise_sigma;
    double dvl_scale;
    double depth_noise_sigma;
    double angle_noise_sigma;
    double recvim_noise_sigma;
    double beacon_depth_noise_sigma;
    // usbl
    bool USBL;
    bool NOISE;
    bool DEPTH;
    Eigen::Vector3d beacon_pos;
    Eigen::Vector3d usbl_rpy;
    

    // dvl 外参数
    Eigen::Matrix3d R_bd; // cam to body
    Eigen::Vector3d t_bd; // cam to body
protected:
    ros::NodeHandle private_nh_;

};
Param::Param(const ros::NodeHandle &private_nh)
    :private_nh_(private_nh)
{
    loadParams();
}

void Param::loadParams()
{
    
    try
    {
        private_nh_.getParam("type", motion_param.type);

        std::cout << "read type; traj type is " << motion_param.type << std::endl;

        private_nh_.getParam("imu_timestep", measurement_param.imu_timestep);
        std::cout << "imu_timestep is " << measurement_param.imu_timestep << std::endl;

        private_nh_.getParam("dvl_timestep", measurement_param.dvl_timestep);
        private_nh_.getParam("depth_timestep", measurement_param.depth_timestep);
        private_nh_.getParam("angle_timestep", measurement_param.angle_timestep);
        private_nh_.getParam("recvim_timestep", measurement_param.recvim_timestep);
        private_nh_.getParam("t_start", motion_param.t_start);
        private_nh_.getParam("t_end", motion_param.t_end);
        private_nh_.getParam("circle_num", motion_param.circle_num);
        private_nh_.getParam("x_mag", motion_param.x_mag);
        private_nh_.getParam("y_mag", motion_param.y_mag);
        private_nh_.getParam("z_mag", motion_param.z_mag);
        private_nh_.getParam("rp_mag", motion_param.rp_mag);
        private_nh_.getParam("yaw_mag", motion_param.yaw_mag);
        private_nh_.getParam("augular_speed", motion_param.augular_speed);

        private_nh_.getParam("FAILURE", failure.Failure);
        private_nh_.getParam("failure_t_start", failure.failure_t_start);
        private_nh_.getParam("failure_t_end", failure.failure_t_end);

        private_nh_.getParam("roll_sigma", imu_param.roll_sigma);
        private_nh_.getParam("pitch_sigma", imu_param.pitch_sigma);
        private_nh_.getParam("yaw_sigma", imu_param.yaw_sigma);

        private_nh_.getParam("gyro_bias_sigma", imu_param.gyro_bias_sigma);
        private_nh_.getParam("acc_bias_sigma", imu_param.acc_bias_sigma);
        private_nh_.getParam("gyro_noise_sigma", imu_param.gyro_noise_sigma);
        private_nh_.getParam("acc_noise_sigma", imu_param.acc_noise_sigma);

        private_nh_.getParam("dvl_noise_sigma", dvl_noise_sigma);
        private_nh_.getParam("dvl_scale", dvl_scale);
        
        private_nh_.getParam("depth_noise_sigma", depth_noise_sigma);
        private_nh_.getParam("angle_noise_sigma", angle_noise_sigma);  
        private_nh_.getParam("recvim_noise_sigma", recvim_noise_sigma); 
        private_nh_.getParam("beacon_depth_noise_sigma", beacon_depth_noise_sigma);  
        XmlRpc::XmlRpcValue usbl_rpy_param;
        private_nh_.getParam("usbl_rpy", usbl_rpy_param);
        std::cout << "usbl_rpy_param is " << usbl_rpy_param[0] << usbl_rpy_param[1] << usbl_rpy_param[2] << std::endl;
        std::cout << "-----------------------------" << std::endl;
        usbl_rpy << usbl_rpy_param[0], usbl_rpy_param[1], usbl_rpy_param[2];   
        
        XmlRpc::XmlRpcValue param;
        private_nh_.getParam("beacon_pos", param);
        std::cout << "param is " << param[0] << param[1] << param[2] << std::endl;
        std::cout << "-----------------------------" << std::endl;
        beacon_pos << param[0], param[1], param[2];   

        private_nh_.getParam("USBL", USBL);
        private_nh_.getParam("NOISE", NOISE);
        private_nh_.getParam("DEPTH", DEPTH);


    }
    catch (...)
    {
        ROS_ERROR("Error loading %s param", "beacon_pos");
    }
}
#endif //IMUSIM_PARAM_H
