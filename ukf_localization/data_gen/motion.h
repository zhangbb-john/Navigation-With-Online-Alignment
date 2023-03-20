//
// Created by hyj on 18-1-19.
//

#ifndef MOTION_H
#define MOTION_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>

#include <data_gen/param.h>

struct MotionData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d eulerAngles;
    Eigen::Vector3d twb;
    Eigen::Vector3d acc;
    Eigen::Vector3d eulerAnglesRates;
    Eigen::Vector3d velocity;
};

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles);
Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles);



class MOTION
{
public:
    MOTION(MotionParam p);
    MotionParam param_;
    Eigen::Vector3d gyro_bias_;
    Eigen::Vector3d acc_bias_;

    Eigen::Vector3d init_velocity_;
    Eigen::Vector3d init_twb_;
    Eigen::Matrix3d init_Rwb_;
    //discrete 
    MotionData discrete_data_;
    bool Initialized_;
    double last_time_;

    MotionData MotionModel(double t);
};

#endif 
