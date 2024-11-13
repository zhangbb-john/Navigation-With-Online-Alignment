#include <ros/ros.h>
#include <sys/time.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <auv_nav_msg/State.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>
#include <utils.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <iomanip>  // std::setprecision()

nav_msgs::Path traj;
nav_msgs::Path truth;
ros::Publisher truthPub;
ros::Publisher trajPub;
ros::Publisher beacon_marker_pub;
std::ofstream traj_file;
std::ofstream truth_file;
std::string log_dir = "/home/ubuntu/Desktop/fusion_localization/Integrated_navigation/src/ukf_localization/log/evo";


void truthCallback(const nav_msgs::Odometry odom){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = odom.header.stamp;
    pose_stamped.header.frame_id = odom.header.frame_id;
    pose_stamped.pose = odom.pose.pose;
    truth.header.stamp = pose_stamped.header.stamp;
    truth.header.frame_id = "temp_odom";
    truth.poses.push_back(pose_stamped);   

    truthPub.publish(truth);

    tf2::Quaternion q(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z,pose_stamped.pose.orientation.w);
    tf2::Matrix3x3 orientation(q);
    double roll, pitch, yaw;
    orientation.getRPY(roll, pitch, yaw);
    truth_file<<std::fixed << std::setprecision(5)<<
        pose_stamped.header.stamp<<" "<<
        pose_stamped.pose.position.x<<" "<<
        pose_stamped.pose.position.y<<" " <<
        pose_stamped.pose.position.z<<" " <<
        roll<<" "<<
        pitch<<" "<<
        yaw<<" "<<
        pose_stamped.pose.orientation.x<<" "<<
        pose_stamped.pose.orientation.y<<" "<<
        pose_stamped.pose.orientation.z<<" "<<
        pose_stamped.pose.orientation.w<<" "<< 
        std::endl;
    //Eigen::Quaternionf q(pose_stamped.pose.orientation.w,pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z);
    //Eigen::Vector3f rpy = q.toRotationMatrix().eulerAngles(2,1,0);
    // std::cout<<   pose_stamped.header.stamp<<" "<<
    //     pose_stamped.pose.orientation.x<<" "<<
    //     pose_stamped.pose.orientation.y<<" "<<
    //     pose_stamped.pose.orientation.z<<" "<<
    //     pose_stamped.pose.orientation.w<<" "<<std::endl;
    // truth_file<<std::fixed << std::setprecision(5)<<
    //     pose_stamped.header.stamp<<" "<<
    //     pose_stamped.pose.position.x<<" "<<
    //     pose_stamped.pose.position.y<<" " <<
    //     pose_stamped.pose.position.z<<" " <<
    //     rpy(2)<<" "<<
    //     rpy(1)<<" "<<
    //     rpy(0)<<" "<<
    //     pose_stamped.pose.orientation.x<<" "<<
    //     pose_stamped.pose.orientation.y<<" "<<
    //     pose_stamped.pose.orientation.z<<" "<<
    //     pose_stamped.pose.orientation.w<<" "<< 
    //     std::endl;
}
void trajCallback(const nav_msgs::Odometry odom){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = odom.header.stamp;
    pose_stamped.header.frame_id = odom.header.frame_id;
    pose_stamped.pose = odom.pose.pose;
    traj.header.stamp = pose_stamped.header.stamp;
    // traj.header.stamp = ros::Time().fromSec(pose_stamped.header.stamp.toSec() - 10);

    traj.header.frame_id = "temp_odom";
    traj.poses.push_back(pose_stamped);
    trajPub.publish(traj);
    tf2::Quaternion q(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z,pose_stamped.pose.orientation.w);
    tf2::Matrix3x3 orientation(q);
    double roll, pitch, yaw;
    orientation.getRPY(roll, pitch, yaw);
    traj_file<<std::fixed << std::setprecision(5)<<
        pose_stamped.header.stamp<<" "<<
        pose_stamped.pose.position.x<<" "<<
        pose_stamped.pose.position.y<<" " <<
        pose_stamped.pose.position.z<<" " <<
        roll<<" "<<
        pitch<<" "<<
        yaw<<" "<<
        pose_stamped.pose.orientation.x<<" "<<
        pose_stamped.pose.orientation.y<<" "<<
        pose_stamped.pose.orientation.z<<" "<<
        pose_stamped.pose.orientation.w<<" "<< 
        std::endl;
    // traj_file<<pose_stamped.header.stamp<<" "<<
    //     pose_stamped.pose.position.x<<" "<<
    //     pose_stamped.pose.position.y<<" " <<
    //     pose_stamped.pose.position.z<<" " <<
    //     pose_stamped.pose.orientation.x<<" "<<
    //     pose_stamped.pose.orientation.y<<" "<<
    //     pose_stamped.pose.orientation.z<<" "<<
    //     pose_stamped.pose.orientation.w<<" "<<std::endl;
    // Eigen::Quaterniond q(pose_stamped.pose.orientation.w,pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z);
    // Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(2,1,0);
    
    // traj_file<<std::fixed << std::setprecision(5) <<  pose_stamped.header.stamp<<" "<<
    //     pose_stamped.pose.position.x<<" "<<
    //     pose_stamped.pose.position.y<<" " <<
    //     pose_stamped.pose.position.z<<" " <<
    //     rpy(2)<<" "<<
    //     rpy(1)<<" "<<
    //     rpy(0)<<" "<<
    //     pose_stamped.pose.orientation.x<<" "<<
    //     pose_stamped.pose.orientation.y<<" "<<
    //     pose_stamped.pose.orientation.z<<" "<<
    //     pose_stamped.pose.orientation.w<<" "<< 
    //     std::endl;
}

void stateCallback(const auv_nav_msg::State msg)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "temp_odom";
    marker.header.stamp = ros::Time();
    marker.ns = "beacon";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = msg.beacon_pos.x;
    marker.pose.position.y = msg.beacon_pos.y;
    marker.pose.position.z = msg.beacon_pos.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    beacon_marker_pub.publish( marker );
}
int main(int argc, char** argv)
{
    std::cout<<"2021/06/09/11:39"<<std::endl;
    ros::init(argc, argv, "traj_pub");
    ros::NodeHandle traj_pub;
    truthPub = traj_pub.advertise<nav_msgs::Path>("truth", 1 , true);  
    trajPub = traj_pub.advertise<nav_msgs::Path> ("traj", 1 , true);
    beacon_marker_pub = traj_pub.advertise<visualization_msgs::Marker> ("beacon_marker", 0 );
    ros::Subscriber pose_truth = traj_pub.subscribe(
        "/Locater/Truth", 1, truthCallback);
    ros::Subscriber traj_truth = traj_pub.subscribe("/Locater/Odom", 1, trajCallback);
    ros::Subscriber state_sub = traj_pub.subscribe("/State", 1, stateCallback);
    
    std::string traj_str = log_dir + "/traj.txt";
    traj_file.open(traj_str.c_str(), std::ios::out);

    std::string truth_str = log_dir + "/truth.txt";
    truth_file.open(truth_str.c_str(), std::ios::out);
    ros::spin();        
    return 0;
}