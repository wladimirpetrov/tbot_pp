#pragma once

#include <iostream>

#define PI 3.14159265
#define THRESHOLD_SWITCH 0.0025
#define dt 0.1
#define MAX_ANGLE_DEVIATION 8  // Max deviation in degrees
#define GRID_SIZE 9
#define GOAL_X 18.5  // Target X position
#define GOAL_Y 2.0  // Target Y position
#define WALL_DETECT_DIST 1.0
#define OPEN_DETECT_DIST 1.5

typedef std::pair<int, int> coord;  

enum { UNDEFINED = -1, NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
enum { OPEN = 0, WALL = 1 };
enum { GOAL_NOT_REACH = 0, GOAL_REACH = 1 };

// Define the goal and starting coordinates
coord GoalCoord(GOAL_X, GOAL_Y);
coord startCoord(0, 0);

// Structure to represent a cell in the map
typedef struct {
  coord position;
  bool wallUp;
  bool wallDown;
  bool wallLeft;
  bool wallRight;
} cell;

// Define the map grid
cell Map[GRID_SIZE][GRID_SIZE];

// Function to calculate Manhattan distance
int manhattanDist(coord start, coord end) {
  int x = abs(start.first - end.first);
  int y = abs(start.second - end.second);
  return x + y;
}
