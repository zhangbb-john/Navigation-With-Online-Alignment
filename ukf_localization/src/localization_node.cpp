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
  startSec = 1632712539.36;
#else

  startSec = ros::Time::now().toSec();
  while (startSec < 1.0e9)
  {
    startSec = ros::Time::now().toSec();
    sleep(1);
  }
  std::cout << "startSec is " << startSec << std::endl;

#endif
  std::string dir;
  ros::param::get("dir", dir);
  File_init(dir);
  LocalizationRos localizationNode(nh, privateNh);
  ros::spin();
  return 0;
}
