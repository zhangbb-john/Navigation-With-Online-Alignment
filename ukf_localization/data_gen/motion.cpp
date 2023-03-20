//
// Created by hyj on 18-1-19.
//

#include <random>
#include "data_gen/motion.h"
#include <iomanip> // std::setprecision()

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb << cy * cp, cy * sp * sr - sy * cr, sy * sr + cy * cr * sp,
        sy * cp, cy * cr + sy * sr * sp, sp * sy * cr - cy * sr,
        -sp, cp * sr, cp * cr;
    return RIb;
}

Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);

    Eigen::Matrix3d R;
    R << 1, 0, -sp,
        0, cr, sr * cp,
        0, -sr, cr * cp;

    return R;
}

MOTION::MOTION(MotionParam p) : param_(p)
{
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();
    Initialized_ = false;
}

MotionData MOTION::MotionModel(double t)
{
    /*10  static; 11 uniform linear motion ; 12 acc ;13 acc then uniform motion ;21 circle sin cos attitude;22.circle fixed angle  ;23 big agnle ;24 +3.14 ; 25 non-uniform circle*/
    if (param_.type == 10)
    {
        MotionData data;

        // twb:  body frame in world frame
        Eigen::Vector3d position(0, 0, 0);
        //std::cout<< std::fixed << std::setprecision(2)<<t<<" "<< 15 * cos( (M_PI / 6) * t ) + 5<<std::endl<<position[0]<<std::endl;

        Eigen::Vector3d dp(0, 0, 0);  // position导数　in world frame
        Eigen::Vector3d ddp(0, 0, 0); // position二阶导数

        Eigen::Vector3d eulerAngles(0, 0, 0);      // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        Eigen::Vector3d eulerAnglesRates(0, 0, 0); // euler angles 的导数

        Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame
        //Eigen::Vector3d gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

        //Eigen::Vector3d gn (0, 0, -9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
        //Eigen::Vector3d acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

        data.eulerAnglesRates = eulerAnglesRates;
        data.acc = ddp;
        data.Rwb = Rwb;
        data.eulerAngles = eulerAngles;
        data.twb = position;
        data.velocity = dp;
        data.timestamp = t;
        return data;
    }
    else if (param_.type == 11)
    {
        MotionData data;

        Eigen::Vector3d position(0.5 * t, 0, 0);

        Eigen::Vector3d dp(0.5, 0, 0); // position导数　in world frame
        Eigen::Vector3d ddp(0, 0, 0);  // position二阶导数

        Eigen::Vector3d eulerAngles(0, 0, 0);      // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        Eigen::Vector3d eulerAnglesRates(0, 0, 0); // euler angles 的导数

        Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame

        data.eulerAnglesRates = eulerAnglesRates;
        data.acc = ddp;
        data.Rwb = Rwb;
        data.eulerAngles = eulerAngles;
        data.twb = position;
        data.velocity = dp;
        data.timestamp = t;
        return data;
    }
    //acc
    else if (param_.type == 12)
    {
        MotionData data;

        Eigen::Vector3d position(0.01 * t * t, 0, 0);

        Eigen::Vector3d dp(0.02 * t, 0, 0); // position导数　in world frame
        Eigen::Vector3d ddp(0.02, 0, 0);    // position二阶导数

        Eigen::Vector3d eulerAngles(0.1, 0.2, 0.3); // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        Eigen::Vector3d eulerAnglesRates(0, 0, 0);  // euler angles 的导数

        Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame

        data.eulerAnglesRates = eulerAnglesRates;
        data.acc = ddp;
        data.Rwb = Rwb;
        data.eulerAngles = eulerAngles;
        data.twb = position;
        data.velocity = dp;
        data.timestamp = t;
        return data;
    }
    else if (param_.type == 13)
    {
        MotionData data;
        if (t < 10)
        {
            Eigen::Vector3d position(0.01 * t * t, 0, 0);

            Eigen::Vector3d dp(0.02 * t, 0, 0); // position导数　in world frame
            Eigen::Vector3d ddp(0.02, 0, 0);    // position二阶导数

            Eigen::Vector3d eulerAngles(0.1, 0.2, 0.3); // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
            Eigen::Vector3d eulerAnglesRates(0, 0, 0);  // euler angles 的导数

            Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame

            data.eulerAnglesRates = eulerAnglesRates;
            data.acc = ddp;
            data.Rwb = Rwb;
            data.eulerAngles = eulerAngles;
            data.twb = position;
            data.velocity = dp;
            data.timestamp = t;
            return data;
        }
        else
        {
            Eigen::Vector3d position(1 + (t - 10) * 0.2, 0, 0);

            Eigen::Vector3d dp(0.2, 0, 0); // position导数　in world frame
            Eigen::Vector3d ddp(0, 0, 0);  // position二阶导数

            Eigen::Vector3d eulerAngles(0.1, 0.2, 0.3); // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
            Eigen::Vector3d eulerAnglesRates(0, 0, 0);  // euler angles 的导数

            Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame

            data.eulerAnglesRates = eulerAnglesRates;
            data.acc = ddp;
            data.Rwb = Rwb;
            data.eulerAngles = eulerAngles;
            data.twb = position;
            data.velocity = dp;
            data.timestamp = t;
            return data;
        }
    }
    else if (param_.type == 13)
    {
        MotionData data;

        Eigen::Vector3d position(0.01 * t * t, 0.005 * t * t, 0);

        Eigen::Vector3d dp(0.02 * t, 0.01 * t, 0); // position导数　in world frame
        Eigen::Vector3d ddp(0.02, 0.01, 0);        // position二阶导数

        Eigen::Vector3d eulerAngles(0.1, 0.2, 0.3); // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        Eigen::Vector3d eulerAnglesRates(0, 0, 0);  // euler angles 的导数

        Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame

        data.eulerAnglesRates = eulerAnglesRates;
        data.acc = ddp;
        data.Rwb = Rwb;
        data.eulerAngles = eulerAngles;
        data.twb = position;
        data.velocity = dp;
        data.timestamp = t;
        return data;
    }
    // sin cos
    else if (param_.type == 21)
    {
        MotionData data;
        // param
        double ellipse_x = param_.x_mag;
        double ellipse_y = param_.y_mag;
        double z = param_.z_mag;                                                 // z轴做sin运动
        double K1 = 10;                                                          // z轴的正弦频率是x，y的k1倍
        double K = 2 * M_PI / (MOTION::param_.t_end / MOTION::param_.circle_num); // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

        // translation
        // twb:  body frame in world frame
        // Eigen::Vector3d position(ellipse_x * cos(K * t) , ellipse_y * sin(K * t), z * sin(K1 * K * t));

        Eigen::Vector3d position(ellipse_x * cos(K * t) - ellipse_x , ellipse_y * sin(K * t), z * sin(K1 * K * t));
        //std::cout<< std::fixed << std::setprecision(2)<<t<<" "<< 15 * cos( (M_PI / 6) * t ) + 5<<std::endl<<position[0]<<std::endl;

        Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), K * ellipse_y * cos(K * t), z * K1 * K * cos(K1 * K * t)); // position导数　in world frame
        double K2 = K * K;
        Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * sin(K * t), -z * K1 * K1 * K2 * sin(K1 * K * t)); // position二阶导数

        // Rotation
        double k_roll = param_.rp_mag;
        double k_pitch = param_.rp_mag;
        double k_yaw = param_.yaw_mag;
        double w = param_.augular_speed / param_.yaw_mag;
        Eigen::Vector3d eulerAngles(k_roll * cos(w * t), k_pitch * sin(w * t), k_yaw * sin(w * t));                   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        Eigen::Vector3d eulerAnglesRates(-k_roll * w * sin(w * t), k_pitch * w * cos(w * t), k_yaw * w * cos(w * t)); // euler angles 的导数

        Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame
        //Eigen::Vector3d gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

        //Eigen::Vector3d gn (0, 0, -9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
        //Eigen::Vector3d acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

        data.eulerAnglesRates = eulerAnglesRates;
        data.acc = ddp;
        data.Rwb = Rwb;
        data.eulerAngles = eulerAngles;
        data.twb = position;
        data.velocity = dp;
        data.timestamp = t;
        return data;
    }
    //sin cos but angle  0.1
    else if (param_.type == 22)
    {
        MotionData data;
        // param
        double ellipse_x = 15;
        double ellipse_y = 20;
        double z = 1;                                                            // z轴做sin运动
        double K1 = 10;                                                          // z轴的正弦频率是x，y的k1倍
        double K = M_PI / (MOTION::param_.t_end / MOTION::param_.circle_num / 2); // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

        // translation
        // twb:  body frame in world frame
        Eigen::Vector3d position(ellipse_x * cos(K * t) - ellipse_x, ellipse_y * sin(K * t), z * sin(K1 * K * t));
        //std::cout<< std::fixed << std::setprecision(2)<<t<<" "<< 15 * cos( (M_PI / 6) * t ) + 5<<std::endl<<position[0]<<std::endl;

        Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), K * ellipse_y * cos(K * t), z * K1 * K * cos(K1 * K * t)); // position导数　in world frame
        double K2 = K * K;
        Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * sin(K * t), -z * K1 * K1 * K2 * sin(K1 * K * t)); // position二阶导数

        // Rotation
        Eigen::Vector3d eulerAngles(0.1, 0.2, 0.3); // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        Eigen::Vector3d eulerAnglesRates(0, 0, 0);  // euler angles 的导数

        Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame
        //Eigen::Vector3d gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

        //Eigen::Vector3d gn (0, 0, -9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
        //Eigen::Vector3d acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

        data.eulerAnglesRates = eulerAnglesRates;
        data.acc = ddp;
        data.Rwb = Rwb;
        data.eulerAngles = eulerAngles;
        data.twb = position;
        data.velocity = dp;
        data.timestamp = t;
        return data;
    }
    //big angle
    else if (param_.type == 23)
    {
        MotionData data;
        // param
        double ellipse_x = 15;
        double ellipse_y = 20;
        double z = 1;                                                            // z轴做sin运动
        double K1 = 10;                                                          // z轴的正弦频率是x，y的k1倍
        double K = M_PI / (MOTION::param_.t_end / MOTION::param_.circle_num / 2); // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

        // translation
        // twb:  body frame in world frame
        Eigen::Vector3d position(ellipse_x * cos(K * t) - ellipse_x, ellipse_y * sin(K * t), z * sin(K1 * K * t));
        //std::cout<< std::fixed << std::setprecision(2)<<t<<" "<< 15 * cos( (M_PI / 6) * t ) + 5<<std::endl<<position[0]<<std::endl;

        Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), K * ellipse_y * cos(K * t), z * K1 * K * cos(K1 * K * t)); // position导数　in world frame
        double K2 = K * K;
        Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * sin(K * t), -z * K1 * K1 * K2 * sin(K1 * K * t)); // position二阶导数

        // Rotation
        double k_roll = 0.5;
        double k_pitch = 0.4;
        double k_yaw = 1;
        Eigen::Vector3d eulerAngles(k_roll * cos(t), k_pitch * sin(t), k_yaw * sin(t));       // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t), k_pitch * cos(t), k_yaw * cos(t)); // euler angles 的导数

        Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame
        //Eigen::Vector3d gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

        //Eigen::Vector3d gn (0, 0, -9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
        //Eigen::Vector3d acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs
        //std::cout<<"motion.cpp line 190 in MotionModel() function :"<<eulerAngles.transpose()<<std::endl;
        data.eulerAnglesRates = eulerAnglesRates;
        data.acc = ddp;
        data.Rwb = Rwb;
        data.eulerAngles = eulerAngles;
        data.twb = position;
        data.velocity = dp;
        data.timestamp = t;
        return data;
    }
    else if (param_.type == 24)
    {
        MotionData data;
        // param
        double ellipse_x = 15;
        double ellipse_y = 20;
        double z = 1;                                                            // z轴做sin运动
        double K1 = 10;                                                          // z轴的正弦频率是x，y的k1倍
        double K = M_PI / (MOTION::param_.t_end / MOTION::param_.circle_num / 2); // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

        // translation
        // twb:  body frame in world frame
        Eigen::Vector3d position(ellipse_x * cos(K * t) - ellipse_x, ellipse_y * sin(K * t), z * sin(K1 * K * t));
        //std::cout<< std::fixed << std::setprecision(2)<<t<<" "<< 15 * cos( (M_PI / 6) * t ) + 5<<std::endl<<position[0]<<std::endl;

        Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), K * ellipse_y * cos(K * t), z * K1 * K * cos(K1 * K * t)); // position导数　in world frame
        double K2 = K * K;
        Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * sin(K * t), -z * K1 * K1 * K2 * sin(K1 * K * t)); // position二阶导数

        // Rotation
        double k_roll = 0.1;
        double k_pitch = 0.2;
        double k_yaw = 0.15;
        Eigen::Vector3d eulerAngles(k_roll * cos(t), k_pitch * sin(t), k_yaw * sin(t) + 3.14); // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t), k_pitch * cos(t), k_yaw * cos(t));  // euler angles 的导数

        Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame
        //Eigen::Vector3d gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

        //Eigen::Vector3d gn (0, 0, -9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
        //Eigen::Vector3d acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

        data.eulerAnglesRates = eulerAnglesRates;
        data.acc = ddp;
        data.Rwb = Rwb;
        data.eulerAngles = eulerAngles;
        data.twb = position;
        data.velocity = dp;
        data.timestamp = t;
        return data;
    }
    else if (param_.type == 25)
    {
        MotionData data;
        // param
        double ellipse_x = 15;
        double ellipse_y = 20;
        double z = 1; // z轴做sin运动
        // double K1 = 10;          // z轴的正弦频率是x，y的k1倍
        // double K = M_PI/ ( MOTION::param_.t_end / MOTION::param_.circle_num / 2);    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

        // translation
        // twb:  body frame in world frame
        Eigen::Vector3d position(ellipse_x * cos(0.01 * t * t) - ellipse_x, ellipse_y * sin(0.01 * t * t), z * sin(0.01 * t * t));
        //std::cout<< std::fixed << std::setprecision(2)<<t<<" "<< 15 * cos( (M_PI / 6) * t ) + 5<<std::endl<<position[0]<<std::endl;

        Eigen::Vector3d dp(-ellipse_x * sin(0.01 * t * t) * 0.02 * t, ellipse_y * cos(0.01 * t * t) * 0.02 * t, z * cos(0.01 * t * t) * 0.02 * t); // position导数　in world frame
        // double K2 = K * K;
        Eigen::Vector3d ddp(-ellipse_x * ((cos(0.01 * t * t) * 0.02 * t) * 0.02 * t + sin(0.01 * t * t) * 0.02), ellipse_y * ((-sin(0.01 * t * t) * 0.02 * t) * 0.02 * t + cos(0.01 * t * t) * 0.02), z * ((-sin(0.01 * t * t) * 0.02 * t) * 0.02 * t + cos(0.01 * t * t) * 0.02)); // position二阶导数

        // Rotation
        double k_roll = 0.1;
        double k_pitch = 0.2;
        double k_yaw = 0.15;
        Eigen::Vector3d eulerAngles(k_roll * cos(t), k_pitch * sin(t), k_yaw * sin(t) + 3.14); // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t), k_pitch * cos(t), k_yaw * cos(t));  // euler angles 的导数

        Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame
        //Eigen::Vector3d gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

        //Eigen::Vector3d gn (0, 0, -9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
        //Eigen::Vector3d acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

        data.eulerAnglesRates = eulerAnglesRates;
        data.acc = ddp;
        data.Rwb = Rwb;
        data.eulerAngles = eulerAngles;
        data.twb = position;
        data.velocity = dp;
        data.timestamp = t;
        return data;
    }
    else if (param_.type == 31)
    {
        double ellipse_x = param_.x_mag;
        double ellipse_y = param_.y_mag;
        double z = param_.z_mag;                                                 // z轴做sin运动
        double K1 = 10;                                                          // z轴的正弦频率是x，y的k1倍
        double K = 2 * M_PI / (MOTION::param_.t_end / MOTION::param_.circle_num); // 20 * K = 2pi 　　由于我们采取的
        double K2 = K * K;
  
        // Rotation
        double k_roll = param_.rp_mag;
        double k_pitch = param_.rp_mag;
        double k_yaw = param_.yaw_mag;
        double w = param_.augular_speed / param_.yaw_mag;

        if (!Initialized_)
        {
            Eigen::Vector3d position(ellipse_x * cos(K * t) - ellipse_x , ellipse_y * sin(K * t), z * sin(K1 * K * t));
            //std::cout<< std::fixed << std::setprecision(2)<<t<<" "<< 15 * cos( (M_PI / 6) * t ) + 5<<std::endl<<position[0]<<std::endl;

            Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), K * ellipse_y * cos(K * t), z * K1 * K * cos(K1 * K * t)); // position导数　in world frame
            Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * sin(K * t), -z * K1 * K1 * K2 * sin(K1 * K * t)); // position二阶导数

            // Rotation
            double k_roll = param_.rp_mag;
            double k_pitch = param_.rp_mag;
            double k_yaw = param_.yaw_mag;
            double w = param_.augular_speed / param_.yaw_mag;
            Eigen::Vector3d eulerAngles(k_roll * cos(w * t), k_pitch * sin(w * t), k_yaw * sin(w * t));                   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
            Eigen::Vector3d eulerAnglesRates(-k_roll * w * sin(w * t), k_pitch * w * cos(w * t), k_yaw * w * cos(w * t)); // euler angles 的导数

            Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame
            //Eigen::Vector3d gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

            //Eigen::Vector3d gn (0, 0, -9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
            //Eigen::Vector3d acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs



            discrete_data_.eulerAnglesRates = eulerAnglesRates;
            discrete_data_.acc = ddp;
            discrete_data_.Rwb = Rwb;
            discrete_data_.eulerAngles = eulerAngles;
            discrete_data_.twb = position;
            discrete_data_.velocity = dp;
            discrete_data_.timestamp = t;
            last_time_ = t;
            Initialized_ = true;
        }
        else
        {
            discrete_data_.eulerAngles = discrete_data_.eulerAngles + discrete_data_.eulerAnglesRates * (t - last_time_);
            Eigen::Matrix3d Rwb = euler2Rotation(discrete_data_.eulerAngles); // body frame to world frame
            discrete_data_.Rwb = Rwb;
            discrete_data_.twb = discrete_data_.twb + discrete_data_.velocity * (t - last_time_);
            discrete_data_.velocity = discrete_data_.velocity + discrete_data_.acc * (t - last_time_);

            Eigen::Vector3d eulerAnglesRates(-k_roll * w * sin(w * t), k_pitch * w * cos(w * t), k_yaw * w * cos(w * t)); // euler angles 的导数
            discrete_data_.eulerAnglesRates = eulerAnglesRates;
            Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * sin(K * t), -z * K1 * K1 * K2 * sin(K1 * K * t)); // position二阶导数
            discrete_data_.acc = ddp;
            discrete_data_.timestamp = t;   
            last_time_ = t;   
        }
        return discrete_data_;
    }
    else
    {
        MotionData data;

        // twb:  body frame in world frame
        Eigen::Vector3d position(0, 0, 0);
        //std::cout<< std::fixed << std::setprecision(2)<<t<<" "<< 15 * cos( (M_PI / 6) * t ) + 5<<std::endl<<position[0]<<std::endl;

        Eigen::Vector3d dp(0, 0, 0);  // position导数　in world frame
        Eigen::Vector3d ddp(0, 0, 0); // position二阶导数

        Eigen::Vector3d eulerAngles(0, 0, 0);      // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
        Eigen::Vector3d eulerAnglesRates(0, 0, 0); // euler angles 的导数

        Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame
        //Eigen::Vector3d gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

        //Eigen::Vector3d gn (0, 0, -9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
        //Eigen::Vector3d acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

        data.eulerAnglesRates = eulerAnglesRates;
        data.acc = ddp;
        data.Rwb = Rwb;
        data.eulerAngles = eulerAngles;
        data.twb = position;
        data.velocity = dp;
        data.timestamp = t;
        return data;
    }
}

// MotionData MOTION::MotionModel(double t)
// {

//     MotionData data;
//     // param
//     float ellipse_x = 15;
//     float ellipse_y = 20;
//     float z = 1;           // z轴做sin运动
//     float K1 = 10;          // z轴的正弦频率是x，y的k1倍
//     float K = M_PI/ ( MOTION::param_.t_end  / MOTION::param_.circle_num / 2);    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

//     // translation
//     // twb:  body frame in world frame
//     Eigen::Vector3d position( ellipse_x * cos( K * t ) + 5, ellipse_y * sin( K * t ) + 5,  z * sin( K1 * K * t ) + 5);
//     Eigen::Vector3d dp(- K * ellipse_x * sin(K*t),  K * ellipse_y * cos(K*t), z*K1*K * cos(K1 * K * t));              // position导数　in world frame
//     double K2 = K * K;
//     Eigen::Vector3d ddp( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), -z*K1*K1*K2 * sin(K1 * K * t));     // position二阶导数

//     // Rotation
//     double k_roll = 0.1;
//     double k_pitch = 0.2;
//     Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
//     Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数

//     Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
//     //Eigen::Vector3d gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

//     //Eigen::Vector3d gn (0, 0, -9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
//     //Eigen::Vector3d acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

//     data.eulerAnglesRates = eulerAnglesRates;
//     data.acc = ddp;
//     data.Rwb = Rwb;
//     data.eulerAngles = eulerAngles;
//     data.twb = position;
//     data.velocity = dp;
//     data.timestamp = t;
//     return data;
// }

// MotionData MOTION::MotionModel(double t)
// {

//     MotionData data;
//     // param
//     float ellipse_x = 15;
//     float ellipse_y = 20;
//     float K = M_PI/ ( MOTION::param_.t_end / 2);    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

//     // translation
//     // twb:  body frame in world frame
//     Eigen::Vector3d position( ellipse_x * cos( K * t ) + 5, ellipse_y * sin( K * t ) + 5,   5);
//     Eigen::Vector3d dp(- K * ellipse_x * sin(K*t),  K * ellipse_y * cos(K*t), 0);              // position导数　in world frame
//     double K2 = K*K;
//     Eigen::Vector3d ddp( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), 0);     // position二阶导数

//     // Rotation
//     Eigen::Vector3d eulerAngles(0 , 0 , 0);   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
//     Eigen::Vector3d eulerAnglesRates(0 ,0 , 0);      // euler angles 的导数

//     Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
//     //Eigen::Vector3d gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

//     //Eigen::Vector3d gn (0, 0, -9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
//     //Eigen::Vector3d acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

//     data.eulerAnglesRates = eulerAnglesRates;
//     data.acc = ddp;
//     data.Rwb = Rwb;
//     data.eulerAngles = eulerAngles;
//     data.twb = position;
//     data.velocity = dp;
//     data.timestamp = t;
//     return data;
// }