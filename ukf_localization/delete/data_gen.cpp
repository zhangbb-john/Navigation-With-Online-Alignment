//
// Created by zbb on 21/3/26.
//

#include <fstream>
#include <sys/stat.h>
#include <data_gen/motion.h>
#include <iomanip>  // std::setprecision()

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
};
struct DepthData
{
    double timestamp;
    double depth;
};
ImuData imu_gen(MOTION motionGen, double t)
{
    MotionData motion = motionGen.MotionModel(t);
    ImuData data;

    //acc
    //Eigen::Vector3d gn (0,0,-9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    //data.acc  = motion.Rwb.transpose() * ( motion.acc -  gn );  //  Rbw * Rwn * gn = gs
    data.acc  = motion.Rwb.transpose() *  motion.acc;
    
    //gyro 
    data.gyro = eulerRates2bodyRates(motion.eulerAngles) * motion.eulerAnglesRates;   //  euler rates trans to body gyro

    // euler_angle
    data.euler_angle = motion.eulerAngles;
    data.timestamp = t;
    return data;
}
DvlData dvl_gen(MOTION motionGen, double t){
    DvlData data;
    MotionData motion = motionGen.MotionModel(t);
    data.vel = motion.velocity;
    data.timestamp = t;
    return data;
}
DepthData depth_gen(MOTION motionGen, double t){
    DepthData data;
    MotionData motion = motionGen.MotionModel(t);
    data.depth = motion.twb(2);
    data.timestamp = t;
    return data;
}


int main(){
    std::cout<<"0326 2047"<<std::endl;

    std::string dir = "/home/john/Desktop/auv_prj/Integrated_navigation/src/ukf_localization/sim_data";
    mkdir(dir.c_str(), 0777);

    // IMU model
    Param params;
    MOTION motionGen(params);

    std::vector< ImuData > imudata;
    std::vector< DvlData > dvldata;
    std::vector< DepthData > depthdata;

    //imu
    std::string imu_file = dir+"/imu.txt";
    std::ofstream save_imu;
    save_imu.open(imu_file.c_str(),std::ios::out);

    for (float t = params.t_start; t<params.t_end;) {
        ImuData data = imu_gen(motionGen, t);\
        imudata.push_back(data);
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(data.euler_angle(0), Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(data.euler_angle(1), Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(data.euler_angle(2), Eigen::Vector3f::UnitZ());
        save_imu<<std::fixed << std::setprecision(5)<<t<<" "
                <<q.coeffs().x()<<" "<<q.coeffs().y()<<" "<<q.coeffs().z()<<" "<<q.coeffs().w()<<" "
                <<data.acc(0)<<" "<<data.acc(1)<<" "<<data.acc(2)<<" "
                <<data.gyro(0)<<" "<<data.gyro(2)<<" "<<data.gyro(2)<<std::endl;

        t += 1.0/params.imu_frequency;
    }
    //dvl
    std::string dvl_file = dir+"/dvl.txt";
    std::ofstream save_dvl;
    save_dvl.open(dvl_file.c_str(),std::ios::out);
    for (float t = params.t_start; t<params.t_end;) {
        DvlData data = dvl_gen(motionGen, t);
        dvldata.push_back(data);
        save_dvl<<std::fixed << std::setprecision(5)<<t<<" "
                <<data.vel(0)<<" "<<data.vel(1)<<" "<<data.vel(2)<<std::endl;
        t += 1.0/params.dvl_frequency;
    }
    //depth
    std::string depth_file = dir+"/depth.txt";
    std::ofstream save_depth;
    save_depth.open(depth_file.c_str(),std::ios::out);   
    for (float t = params.t_start; t<params.t_end;) {
        DepthData data = depth_gen(motionGen, t);

        depthdata.push_back(data);
        save_depth<<std::fixed << std::setprecision(5)<<t<<" "
                <<data.depth<<std::endl;
        t += 1.0/params.depth_frequency;
    }
    return 0;
}
