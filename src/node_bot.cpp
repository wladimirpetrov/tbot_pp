#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

const double MAX_LINEAR_VEL = 0.7;   // Maximum linear velocity
const double MAX_ANGULAR_VEL = M_PI; // Maximum angular velocity

double target_x, target_y, goal_reached = 0;
void plannerCallback(const std_msgs::Float64MultiArray::ConstPtr target_msg)
{
  target_x = target_msg->data[0];
  target_y = target_msg->data[1];
  goal_reached = target_msg->data[2];
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
  ros::init(argc, argv, "node_bot"); 
  ros::NodeHandle nh;

  double Kp_a, Ki_a, Kd_a, Kp_x, Ki_x, Kd_x, dt;
  if (!nh.getParam("Kp_a", Kp_a) ||
      !nh.getParam("Ki_a", Ki_a) ||
      !nh.getParam("Kd_a", Kd_a) ||
      !nh.getParam("Kp_x", Kp_x) ||
      !nh.getParam("Ki_x", Ki_x) ||
      !nh.getParam("Kd_x", Kd_x) ||
      !nh.getParam("dt", dt))
  {
    ROS_ERROR("Failed to load parameters");
    return 1;
  }

  ros::Subscriber sub_planner = nh.subscribe<std_msgs::Float64MultiArray>("/planner", 1, plannerCallback);
  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("/odom", 1, odomCallback);
  ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  geometry_msgs::Twist msg_cmd;

  ROS_INFO("Waiting for messages...");
  while (ros::ok() && std::isnan(heading))
  {
    ros::spinOnce();
  }
  ROS_INFO("Initialized.");

  double error_pos = 0, error_pos_prev, error_heading_prev, error_heading;
  double error_x, error_y;
  double vel_x = 0, vel_heading = 0;
  double target_heading;
  double I_angular, I_linear, D_angular, D_pos;
  ros::Rate rate(1 / dt);

  while (ros::ok() && goal_reached == 0)
  {
    ros::spinOnce();

    error_pos_prev = error_pos;
    error_heading_prev = error_heading;

    error_x = target_x - pos_x;
    error_y = target_y - pos_y;
    error_pos = sqrt(error_x * error_x + error_y * error_y);
    target_heading = atan2(error_y, error_x);
    error_heading = target_heading - heading;

    if (error_heading < -M_PI)
      error_heading += 2 * M_PI;
    if (error_heading > M_PI)
      error_heading -= 2 * M_PI;

    I_angular = dt * error_heading;
    I_linear = dt * error_pos;
    D_angular = (error_heading_prev - error_heading) / dt;
    D_pos = (error_pos_prev - error_pos) / dt;

    vel_heading = (Kp_a * error_heading) + (Ki_a * I_angular) + (Kd_a * D_angular);
    vel_x = (Kp_x * error_pos) + (Ki_x * I_linear) + (Kd_x * D_pos);

    if (vel_x > MAX_LINEAR_VEL)
      vel_x = MAX_LINEAR_VEL;
    if (vel_heading > MAX_ANGULAR_VEL)
      vel_heading = MAX_ANGULAR_VEL;

    msg_cmd.linear.x = vel_x;
    msg_cmd.angular.z = vel_heading;
    pub_cmd.publish(msg_cmd);

    rate.sleep();
  }

  msg_cmd.linear.x = 0;
  msg_cmd.angular.z = 0;
  pub_cmd.publish(msg_cmd);

  ros::Duration(1.0).sleep();

  return 0;
}
