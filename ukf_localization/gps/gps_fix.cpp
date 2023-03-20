#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <iomanip>
#define ALIGN
double start_sec = 0;
double last_lat = 0, last_lon = 0, n = 0, e = 0, odom_n = 0, odom_e = 0;
bool INITED = false, GOT_ODOM = false;
double last_measurementtime;
std::ofstream logfile;
std::string dir;
ros::Publisher truth_pub;
void gpsCallback(const nav_msgs::Odometry msg)
{
    if (msg.pose.pose.position.x > 0 && msg.pose.pose.position.z > 5)
    {
        double lat, lon;
        std::cout << "time now is " << ros::Time::now().toSec() - start_sec << ";receive gps, Inited is " << INITED << ";n is " << n << "e is " << e << "last lat is :" << last_lat << ";last lon is " << last_lon << std::endl;
        static tf::TransformBroadcaster br;
        lat = msg.pose.pose.position.x / 180 * M_PI;
        lon = msg.pose.pose.position.y / 180 * M_PI;
        if (lat > 0 & lon > 0)
        {
            std::cout << "lat is " << lat << " ; lon is " << lon << std::endl;
#ifdef ALIGN
            if (!INITED && GOT_ODOM)
            {
                INITED = true;
                n = odom_n;
                e = odom_e;
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(n, e, 0.0));
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "temp_odom", "gps"));
                last_lat = lat;
                last_lon = lon;
            }
#else
            if (!INITED)
            {

                n = 0;
                e = 0;
                INITED = true;

                tf::Transform transform;
                transform.setOrigin(tf::Vector3(n, e, 0.0));
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "temp_odom", "gps"));
            }
#endif
            else if (INITED)
            {
                double Re = 6378137;
                double f = 1 / 298.257; // reference ellipsoid
                double Rm = Re * (1 - 2 * f + 2 * f * pow(sin(lat), 2));
                double Rn = Re * (1 + f * pow(sin(lat), 2));
                double diffe = e + (lon - last_lon) * Rn * cos(lat) - odom_e;
                double diffn = n + (lat - last_lat) * Rm - odom_n;
                if (sqrt(diffe * diffe + diffn * diffn) < 50)
                {
                    e = e + (lon - last_lon) * Rn * cos(lat);
                    n = n + (lat - last_lat) * Rm; //% seems to cumsum

                    // 初始化tf数据
                    tf::Transform transform;
                    transform.setOrigin(tf::Vector3(n, e, 0.0));
                    tf::Quaternion q;
                    q.setRPY(0, 0, 0);
                    transform.setRotation(q);

                    // 广播world与海龟坐标系之间的tf数据
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "temp_odom", "gps"));
                    logfile << std::fixed << std::setprecision(10) << n << " " << e << " " << lat << " " << lon << std::endl;

                    nav_msgs::Odometry odom_truth;
                    odom_truth.pose.pose.position.x = n;
                    odom_truth.pose.pose.position.y = e;
                    odom_truth.pose.pose.position.z = msg.pose.pose.position.z;
                    odom_truth.header.stamp = msg.header.stamp;
                    odom_truth.header.frame_id = "temp_odom";
                    truth_pub.publish(odom_truth);
                    last_lat = lat;
                    last_lon = lon;
                }

            }

            std::cout << "n is " << n << " ; e is " << e << std::endl;
        }
    }
}
void odomCallback(const nav_msgs::Odometry odom)
{

    odom_n = odom.pose.pose.position.x;
    odom_e = odom.pose.pose.position.y;
    GOT_ODOM = true;
}
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "gps_fix");

    ros::NodeHandle node;
    ros::Subscriber gps_sub = node.subscribe("/Sensor/Gps", 10, &gpsCallback);
    ros::Subscriber odom_sub = node.subscribe("/Locater/Odom", 10, odomCallback);
    ros::param::get("dir", dir);
    // 循环等待回调函数
    start_sec = ros::Time::now().toSec();
    std::string filename(dir + "/log/gps_log.txt");
    logfile.open(filename.c_str(), std::ios::out);
    truth_pub = node.advertise<nav_msgs::Odometry>("/Locater/Truth", 1, true);
    ros::spin();

    return 0;
};
