#include <localization_ros.h>
#include <ros/ros.h>
#include <sys/time.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization_node");
  std::cout << "20210622 13:34" << std::endl;
  ros::NodeHandle nh;
  ros::NodeHandle privateNh("~");

#ifdef BAG
  start_time = 1632712539.36;
#else

  start_time = ros::Time::now().toSec();
  while (start_time < 1.0e9)
  {
    start_time = ros::Time::now().toSec();
    sleep(1);
  }
  std::cout << "start_time is " << start_time << std::endl;

#endif
  std::string dir;
  ros::param::get("dir", dir);
  fileInit(dir);
  LocalizationRos localizationNode(nh, privateNh);
  ros::spin();
  return 0;
}
