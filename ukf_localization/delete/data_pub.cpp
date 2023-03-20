#include <sys/time.h>
#include <iostream> 
#include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/FluidPressure.h>
#include "auv_nav_msg/DVL.h"
#include <sensor_msgs/Imu.h>

int main(int argc, char** argv)
{
    std::cout<<"21/03/30/16:53"<<std::endl;
    const std::string dir_path = "/home/john/Desktop/auv_prj/Integrated_navigation/src/ukf_localization/sim_data/";
    const std::string imu_path = dir_path+ "imu.txt";
    const std::string dvl_path = dir_path+"dvl.txt";
    const std::string depth_path = dir_path+"depth.txt";

    std::ifstream if_imu(imu_path);
    std::ifstream if_dvl(dvl_path);
    std::ifstream if_depth(depth_path);
    std::cout << "Start generate data, please waiting..." << std::endl;
       
    ros::init (argc, argv, "data_pub");

	ros::NodeHandle data_pub;

	ros::Publisher imu_pub = data_pub.advertise<sensor_msgs::Imu>("/Sensor/AHRS",1, true);
    ros::Publisher dvl_pub = data_pub.advertise<auv_nav_msg::DVL>("/Sensor/DVL",1, true);
    ros::Publisher depth_pub = data_pub.advertise<sensor_msgs::FluidPressure>("/Sensor/Pressure",1,true);

    sensor_msgs::Imu imu;
    auv_nav_msg::DVL dvl;
    sensor_msgs::FluidPressure depth;
    uint count = 0;
    double sec;
    while (1)
    {
        usleep(10000);
        count ++;
        if( count % 2 ==0 )
        {
            if(if_imu >> sec
                   >> imu.orientation.x >> imu.orientation.y >> imu.orientation.z >> imu.orientation.w
                   >> imu.linear_acceleration.x >> imu.linear_acceleration.y >> imu.linear_acceleration.z
                   >> imu.angular_velocity.x >> imu.angular_velocity.y >> imu.angular_velocity.z)
            {
                ros::Time time(sec);
                imu.header.stamp = ros::Time::now() ;
                imu.header.frame_id = "base_link2";
                imu.orientation_covariance = {1e-6,0,0,0,1e-6,0,0,0,1e-6};
                imu.linear_acceleration_covariance = {1e-6,0,0,0,1e-6,0,0,0,1e-6};
                imu.angular_velocity_covariance = {1e-6,0,0,0,1e-6,0,0,0,1e-6};
                imu_pub.publish(imu);
                //ROS_INFO("imu published \n stamp: %f",sec);
                //ROS_INFO("[ x: %f y: %f z: %f]\n", imu.linear_acceleration.x,imu.orientation.y ,imu.orientation.z );
            }
            else{
                ROS_INFO("imu break now");
                break;
            }
        }
        if( count % 25 == 0 ){
            if( if_dvl >> sec >> dvl.velocity_body.x >> dvl.velocity_body.y >> dvl.velocity_body.z )
            {
                ros::Time time(sec);
                dvl.header.stamp = ros::Time::now() ;   
                dvl.header.frame_id = "base_link2" ;
                dvl.velocity_covariance = {1e-6,0,0,0,1e-6,0,0,0,1e-6};
                dvl_pub.publish(dvl);  
                ROS_INFO("dvl published \n stamp: %f",sec);
            }  
            else
            {
                ROS_INFO("dvl break now\n");
                break;
            }        
        }
        if( count % 25 == 0 ){
            if( if_depth >> sec >> depth.fluid_pressure)
            {
                ros::Time time(sec);
                depth.header.stamp = ros::Time::now() ;
                depth.header.frame_id = "base_link2" ;    
                depth.variance = 1e-6;       
                depth_pub.publish(depth);  
                ROS_INFO("depth published \n stamp: %f",sec);

            }
            else{
                ROS_INFO("depth break now");
                break;
            }          
        }
    }

    return 0;
}
