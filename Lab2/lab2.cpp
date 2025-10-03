#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "utils.h"
#include "render.h"

//===== Main parameters =====
const int width{800}, height{800};       // Width and height of the environment
const int radius{10};                    // Radius of the robot's circular body
const int min_obj_size{50};              // Maximum object dimension
const int max_obj_size{100};             // Maximum object dimension
const int goal_width{100};               // Goal width
const int goal_height{100};              // Goal heigth
const int robot_tol{50};                 // Tolerance for robot spawn point
const int occupancy_tol{30};             // Minimum distance between all objects that spawn
const int goal_tol{50};                  // Minimum distance in x,y between robot and goal
const int robot_y_min{500};              // Minimum robot y position
const int goal_y_max{300};               // Maximum goal y position
int obj_x, obj_y, obj_width, obj_height; // Parameters for object position/size
int num_objects{15};                     // Number of objects in environment

// Grid utility class
grid_util grid(width, height, min_obj_size, max_obj_size);

// Random generator
random_generator rand_gen;

// Vector of velocity commands
std::vector<std::vector<int>> robot_pos;

// Did mission succeed?
bool succeed;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++WRITE ANY FUNCTIONS OR GLOBAL VARIABLES HERE+++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Task 2
bool is_goal_detected(const Object &robot, const Object &goal)
{
    // Define robot corner positions
    int topLeftX = robot.x - radius;
    int topLeftY = robot.y - radius;

    int topRightX = robot.x + radius;
    int topRightY = robot.y - radius;

    int bottomLeftX = robot.x - radius;
    int bottomLeftY = robot.y + radius;

    int bottomRightX = robot.x + radius;
    int bottomRightY = robot.y + radius;

    // Goal boundaries
    int goalLeft = goal.x;
    int goalRight = goal.x + goal_width;
    int goalTop = goal.y;
    int goalBottom = goal.y + goal_height;

    // Check if any robot corner is inside the goal rectangle
    auto in_goal = [&](int x, int y)
    {
        return x >= goalLeft && x <= goalRight && y >= goalTop && y <= goalBottom;
    };

    if (in_goal(topLeftX, topLeftY))
        return true;
    if (in_goal(topRightX, topRightY))
        return true;
    if (in_goal(bottomLeftX, bottomLeftY))
        return true;
    if (in_goal(bottomRightX, bottomRightY))
        return true;

    return false; // None of the corners are inside the goal
}

bool isObstacleDetected(const Object &robot)
{
    int robotLeft = (robot.x - radius);
    int robotRight = (robot.x + radius);
    int robotTop = (robot.y - radius);
    int robotBottom = (robot.y + radius);
    // Check Top Left
    if (grid.grid[robotLeft][robotTop] == 2)
    {
        return true;
    }
    // Check Bottom Left
    if (grid.grid[robotLeft][robotBottom] == 2)
    {
        return true;
    }
    // Check Top Right
    if (grid.grid[robotRight][robotTop] == 2)
    {
        return true;
    }
    // Check Bottom Right
    if (grid.grid[robotRight][robotBottom] == 2)
    {
        return true;
    }
    return false;
}

void obstacleAvoidance(Object &robot, std::string direction)
{
    while (isObstacleDetected(robot))
    {
        if (direction == "X")
        {
            robot.y += 1;
            // direction == "Y";
        }
        if (direction == "Y")
        {
            robot.x += 1;
            // direction == "X";
        }

        robot_pos.push_back({robot.x, robot.y});
    }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char const *argv[])
{
    //==========CREATE ROBOT, GOAL, OBJECTS==========
    // create robot
    Object robot = grid.create_object(grid, rand_gen, robot_tol, 2 * radius, 2 * radius, robot_y_min, height - radius, 1, "robot");

    // create the goal
    Object goal = grid.create_object(grid, rand_gen, goal_tol, goal_width, goal_height, 0, goal_y_max, 3, "goal");

    // create the objects
    std::vector<Object> objects = grid.create_objects(rand_gen, occupancy_tol, num_objects);

    // create copies of robot and goal with their initial positions for purpose of render functions
    Object robot_init = robot;
    Object goal_init = goal;
    // also create a copy for predicting collisions
    Object robot_copy = robot;

    // Uncomment this line to write the grid to csv to see the grid as a csv
    // grid.writeGridToCSV("grid.csv");

    robot_pos.push_back({robot.x, robot.y});

    // maximum count. Close the loop after 3600 iterations. As the window is displayed at 60fps, this is 60 seconds.
    int max_count = 0;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+++++++++++++++DEFINE ANY LOCAL VARIABLE HERE+++++++++++++++++++++++
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Task 1
    int robotLeft = (robot.x - radius);
    int robotRight = (robot.x + radius);
    int robotTop = (robot.y - radius);
    int robotBottom = (robot.y + radius);

    // Task 2
    int goalLeft = goal.x,
        goalRight = goal.x + goal_width,
        goalTop = goal.y,
        goalBotton = goal.y + goal_height;

    // Task 2 of Lab 2
    std::string direction="X";
    // main loop
    while (true)
    {
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //++++++++++++++WRITE YOUR CODE HERE++++++++++++++++++++++++++++++++++
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        int robotLeft = (robot.x - radius);
        int robotRight = (robot.x + radius);
        int robotTop = (robot.y - radius);
        int robotBottom = (robot.y + radius);

        // Task 1 (Check if robot hit boundary)
        if ((robotLeft) <= 0 || (robotRight) >= width || (robotTop) <= 0 || (robotBottom) >= height)
        {
            succeed = false;
            std::cout << "Robot hit boundary" << std::endl;
            break;
        }
        // Task 2: Check if robot reached goal
        if (is_goal_detected(robot, goal))
        {
            succeed = true;
            std::cout << "Robot reached the goal!" << std::endl;
            break;
        }
        // Task 3 (1 Direction Pathfinding-X Movement First)
        int deltaY = goal.y - robot.y;
        int deltaX = goal.x - robot.x;
        if (deltaX != 0)
        {
            direction = "X";
            robot.x += 1 * ((deltaX > 0) - (deltaX < 0));
        }
        else if (deltaY != 0)
        {
            direction = "Y";
            robot.y += 1 * ((deltaY > 0) - (deltaY < 0));
        }

        // Task 1 Of Lab 2 (Obstacle Detecting)
        // if (isObstacleDetected(robot))
        // {
        //     succeed = false;
        //     std::cout << "Robot hit Obstacle" << std::endl;
        //     break;
        // }
        
        // Task 2 Of Lab 2 (Obstacle Avoidance)
        if (isObstacleDetected(robot))
        {
            obstacleAvoidance(robot, direction);
        }
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //++++++++++++++++END YOUR CODE HERE++++++++++++++++++++++++++++++++++
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // place the current robot position at the time step to robot_pos
        robot_pos.push_back({robot.x, robot.y});
        max_count++;

        // if more than a minute passed (in render window), exit
        if (max_count >= 3600)
        {
            std::cout << "=====1 minute reached with no solution=====" << std::endl;
            break;
        }
    }

    // send the results of the code to the renderer
    render_window(robot_pos, objects, robot_init, goal_init, width, height, succeed);
    return 0;
}
