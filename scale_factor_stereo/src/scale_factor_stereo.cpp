#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <string>
#include<algorithm>
#include <sstream>
#include <cstdlib>
#include <math.h>
#include <std_msgs/Float32.h>
#include <string>
#include "std_msgs/String.h"

#define EXIT_FILE_ERROR (1)
#define EXIT_UNEXPECTED_EOF (2)
#define EXIT_INVALID_FIRSTLINE (3)
#define MAXLINE (10000)
float input_baseline();
float x_left, y_left, z_left, x_right, y_right, z_right;
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  x_left=msg->pose.pose.position.x;
  y_left=msg->pose.pose.position.y;
  z_left=msg->pose.pose.position.z;

  //ROS_INFO ("x=%f, y=%f", x,y);
}
void chatterCallback2(const nav_msgs::Odometry::ConstPtr& msg)
{
  x_right=msg->pose.pose.position.x;
  y_right=msg->pose.pose.position.y;
  z_right=msg->pose.pose.position.z;
}
using namespace std;
std_msgs::Float32 scale_factor;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "scale_factor_stereo");
  ros::NodeHandle n;
  // input the calibrated baseline from a file 
  float baseline=input_baseline();
  ROS_INFO ("baseline=%f", baseline);
  // subscribe odom node to have x, y, yaw 
  ros::Subscriber sub = n.subscribe("/vo_left", 100,chatterCallback); 
  ros::Subscriber sub2 = n.subscribe("/vo_right", 100, chatterCallback2); 
  ros::Publisher SF_publisher = n.advertise<std_msgs::Float32>("scale_factor", 100);
  scale_factor.data = baseline/sqrt(pow((x_left-x_right),2) + pow((y_left-y_right),2) + pow((z_left-z_right),2));
  SF_publisher.publish(scale_factor);
  ros::spin();
  return 0;
}

float input_baseline()
{
FILE *fp;
  string sFileName = "/home/ali/mybot_ws/src/scale_factor_stereo/calibration_files/base_line.txt";
  ifstream fileStream(sFileName.c_str());
  if (!fileStream.is_open())
  {
	ROS_INFO("Exiting unable to open file" );
	exit(EXIT_FILE_ERROR);
  }
  std::string line;
  getline(fileStream, line);
  float bl=::strtod(line.c_str(), 0);
  return bl;
	
  fileStream.close();
  if ((fp = fopen(sFileName.c_str(), "r")) == NULL)
  {
	ROS_INFO("Exiting unable to open file");
	exit(EXIT_FILE_ERROR);
  }
}

