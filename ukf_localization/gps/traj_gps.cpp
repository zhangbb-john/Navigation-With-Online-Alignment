#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

nav_msgs::Path traj;


int main(int argc, char** argv)
{
    std::cout<<"2021/010/30/11:39"<<std::endl;
    ros::init(argc, argv, "traj_gps");
    ros::NodeHandle traj_gps;
    ros::Publisher traj_pub = traj_gps.advertise<nav_msgs::Path> ("traj_gps", 1 , true);  
    tf::TransformListener listener;
    ros::Rate rate(2.0);
    while(traj_gps.ok())
    {
        tf::StampedTransform transform;
        try {
            listener.waitForTransform("temp_odom", "gps" , ros::Time(0), ros::Duration(3.0) );
            listener.lookupTransform("temp_odom", "gps", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "temp_odom";      
        pose_stamped.pose.position.x = transform.getOrigin().x();
        pose_stamped.pose.position.y = transform.getOrigin().y();
        pose_stamped.pose.position.z = transform.getOrigin().z();
        pose_stamped.pose.orientation.w = transform.getRotation().w();
        pose_stamped.pose.orientation.x = transform.getRotation().x();
        pose_stamped.pose.orientation.y = transform.getRotation().y();
        pose_stamped.pose.orientation.z = transform.getRotation().z();
        traj.poses.push_back(pose_stamped);
        traj.header.stamp = pose_stamped.header.stamp;
        traj.header.frame_id = pose_stamped.header.frame_id ;
        traj_pub.publish(traj);  
        rate.sleep();      

    }      
    return 0;
}

