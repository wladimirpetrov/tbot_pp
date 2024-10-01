#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <deque>

const int MAP_MAX_X = 10;
const int MAP_MAX_Y = 10;

struct Cell
{
  bool walls[4] = {false}; 
  bool north_deadend = false;
};
Cell cells[MAP_MAX_X][MAP_MAX_Y];

double pos_x, pos_y = NAN;
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  pos_x = msg->pose.pose.position.x;
  pos_y = msg->pose.pose.position.y;
}

double north_scan_range, west_scan_range, south_scan_range, east_scan_range, northwest_scan_range, northeast_scan_range = NAN;
void rangesCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  north_scan_range = msg->data[0];
  west_scan_range = msg->data[1];
  south_scan_range = msg->data[2];
  east_scan_range = msg->data[3];
  northwest_scan_range = msg->data[4];
  northeast_scan_range = msg->data[5];
}

std::vector<std::array<int, 2>> findPath(int idx_x, int idx_y, int goal_x, int goal_y)
{
  std::vector<std::array<int, 2>> path;

  struct Node
  {
    int x = -1, y = -1;
    int cost = 10000;
    Node *parent = nullptr;
  };
  Node nodes[MAP_MAX_X][MAP_MAX_Y];
  for (int x = 0; x < MAP_MAX_X; ++x)
  {
    for (int y = 0; y < MAP_MAX_Y; ++y)
    {
      nodes[x][y].x = x;
      nodes[x][y].y = y;
    }
  }

  nodes[idx_x][idx_y].cost = 0;
  std::deque<Node *> queue;
  queue.push_back(&nodes[idx_x][idx_y]);

  while (!queue.empty())
  {
    Node *cur_node = queue.front();
    queue.pop_front();
    int cur_x = cur_node->x;
    int cur_y = cur_node->y;
    int cur_cost = cur_node->cost;

    if (cur_x == goal_x && cur_y == goal_y)
    {
      Node *node = cur_node;
      do
      {
        path.push_back({node->x, node->y});
        node = node->parent;
      } while (node != nullptr);
      break;
    }

    Cell *cur_cell = &cells[cur_x][cur_y];

    if (!cur_cell->walls[0] && cur_x < MAP_MAX_X - 1)
    {
      Node *north_node = &nodes[cur_x + 1][cur_y];
      int new_cost = cur_cost + 1;
      if (north_node->cost > new_cost)
      {
        north_node->cost = new_cost;
        north_node->parent = cur_node;
        queue.push_back(north_node);
      }
    }

    if (!cur_cell->walls[1] && cur_y < MAP_MAX_Y - 1)
    {
      Node *west_node = &nodes[cur_x][cur_y + 1];
      int new_cost = cur_cost + 1;
      if (west_node->cost > new_cost)
      {
        west_node->cost = new_cost;
        west_node->parent = cur_node;
        queue.push_back(west_node);
      }
    }

    if (!cur_cell->walls[2] && cur_x > 0)
    {
      Node *south_node = &nodes[cur_x - 1][cur_y];
      int new_cost = cur_cost + 1;
      if (south_node->cost > new_cost)
      {
        south_node->cost = new_cost;
        south_node->parent = cur_node;
        queue.push_back(south_node);
      }
    }

    if (!cur_cell->walls[3] && cur_y > 0)
    {
      Node *east_node = &nodes[cur_x][cur_y - 1];
      int new_cost = cur_cost + 1;
      if (east_node->cost > new_cost)
      {
        east_node->cost = new_cost;
        east_node->parent = cur_node;
        queue.push_back(east_node);
      }
    }
  }

  return path;
}

void fillWalls(int idx_x, int idx_y)
{
  // Process scan data and determine if there are walls around the robot
  cells[idx_x][idx_y].walls[0] = north_scan_range <= 1;
  cells[idx_x][idx_y].walls[1] = west_scan_range <= 1;
  cells[idx_x][idx_y].walls[2] = south_scan_range <= 1;
  cells[idx_x][idx_y].walls[3] = east_scan_range <= 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pp_node"); 
  ros::NodeHandle nh;

  ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("/odom", 1, odomCallback);
  ros::Subscriber sub_ranges_sub = nh.subscribe<std_msgs::Float64MultiArray>("/obstacles", 1, rangesCallback);
  ros::Publisher pub_planner = nh.advertise<std_msgs::Float64MultiArray>("/planner", 1, true);
  std_msgs::Float64MultiArray msg_planner;
  msg_planner.data.resize(3);

  double pos_goal_x, pos_goal_y;
  if (!nh.getParam("goal_x", pos_goal_x) || !nh.getParam("goal_y", pos_goal_y))
  {
    ROS_ERROR("Failed to load goal parameters");
    return 1;
  }

  ROS_INFO("Waiting for messages...");
  while (ros::ok() && (std::isnan(pos_y) || std::isnan(east_scan_range)))
  {
    ros::spinOnce();
  }
  ROS_INFO("Initialized.");

  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    int idx_x = std::floor(pos_x);
    int idx_y = std::floor(pos_y);

    fillWalls(idx_x, idx_y);

    std::vector<std::array<int, 2>> path = findPath(idx_x, idx_y, std::floor(pos_goal_x), std::floor(pos_goal_y));

    if (!path.empty())
    {
      double target_x = path.size() > 1 ? path[path.size() - 2][0] + 0.5 : pos_goal_x;
      double target_y = path.size() > 1 ? path[path.size() - 2][1] + 0.5 : pos_goal_y;
      msg_planner.data[0] = target_x;
      msg_planner.data[1] = target_y;
      pub_planner.publish(msg_planner);
    }

    rate.sleep();
  }

  return 0;
}
