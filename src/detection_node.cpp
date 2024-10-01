#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

double goal_reached = NAN;
void plannerCallback(const std_msgs::Float64MultiArray::ConstPtr msg)
{
  goal_reached = msg->data[2];
}

std::vector<float> scan_ranges;
void scanCallback(const sensor_msgs::LaserScan::ConstPtr msg)
{
  scan_ranges = msg->ranges;
}

double pos_x, pos_y, heading = NAN;
void odomCallback(const nav_msgs::Odometry::ConstPtr msg)
{
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  pos_x = msg->pose.pose.position.x;
  pos_y = msg->pose.pose.position.y;
  heading = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qz * qz + qy * qy));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_node"); 
  ros::NodeHandle nh;

  ros::Subscriber sub_scan = nh.subscribe("/scan", 1, &scanCallback);
  ros::Subscriber sub_odom = nh.subscribe("/odom", 1, &odomCallback);
  ros::Subscriber sub_planner = nh.subscribe<std_msgs::Float64MultiArray>("/planner", 1, plannerCallback);
  ros::Publisher pub_obs = nh.advertise<std_msgs::Float64MultiArray>("/obstacles", 1);
  std_msgs::Float64MultiArray msg_obs;
  msg_obs.data.resize(6);

  ROS_INFO("Waiting for messages...");
  while (ros::ok() && (std::isnan(goal_reached) || std::isnan(heading) || scan_ranges.empty()))
  {
    ros::spinOnce();
  }
  ROS_INFO("Initialized.");

  int north_idx, west_idx, south_idx, east_idx, northwest_idx, northeast_idx;
  ros::Rate rate(50);
  while (ros::ok() && goal_reached == 0)
  {
    ros::spinOnce();

    north_idx = 0;
    west_idx = 90;
    south_idx = 180;
    east_idx = 270;
    northwest_idx = 20;
    northeast_idx = 340;
    int rotate = heading / M_PI * 180;

    north_idx = (north_idx - rotate + 360) % 360;
    west_idx = (west_idx - rotate + 360) % 360;
    south_idx = (south_idx - rotate + 360) % 360;
    east_idx = (east_idx - rotate + 360) % 360;
    northwest_idx = (northwest_idx - rotate + 360) % 360;
    northeast_idx = (northeast_idx - rotate + 360) % 360;

    msg_obs.data[0] = scan_ranges[north_idx];
    msg_obs.data[1] = scan_ranges[west_idx];
    msg_obs.data[2] = scan_ranges[south_idx];
    msg_obs.data[3] = scan_ranges[east_idx];
    msg_obs.data[4] = scan_ranges[northwest_idx];
    msg_obs.data[5] = scan_ranges[northeast_idx];
    pub_obs.publish(msg_obs);

    rate.sleep();
  }

  return 0;
}
