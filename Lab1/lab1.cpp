#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "utils.h"
#include "render.h"
using namespace std;

//===== Main parameters =====
const int width{800}, height{800};       // Width and height of the environment
const int radius{10};                    // Radius of the robot's circular body
const int min_obj_size{50};              // Maximum object dimension
const int max_obj_size{100};             // Maximum object dimension
const int goal_width{100};               // Goal width
const int goal_height{100};              // Goal heigth
const int robot_tol{200};                // Tolerance for robot spawn point
const int occupancy_tol{50};             // Minimum distance between all objects that spawn
const int goal_tol{100};                 // Minimum distance in x,y between robot and goal
const int robot_y_min{500};              // Minimum robot y position
const int goal_y_max{300};               // Maximum goal y position
int obj_x, obj_y, obj_width, obj_height; // Parameters for object position/size
int num_objects{0};                      // Number of obstacles in environment

// Grid utility class. Students will not use this for lab 1
grid_util grid(width, height, min_obj_size, max_obj_size);

// Random generator to spawn robot and goal
random_generator rand_gen;

// Vector of robot positions to pass to renderer code. Update this after each time step!
std::vector<std::vector<int>> robot_pos;

// Did mission succeed? Update this to make sure it succeeds if robot reaches goal, failure if it hits wall.
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

    // uncomment this line to write the grid to csv to see the grid as a csv
    // grid.writeGridToCSV("grid.csv");

    // place the first robot position to robot_pos
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
    // Task 3-5
    int deltaY = goal.y - robot.y;
    int deltaX = goal.x - robot.x;

    // Task 6 (To goal center)
    int goalCenterY = goal.y + goal_height / 2;
    int goalCenterX = goal.x + goal_width / 2;
    deltaY = goalCenterY - robot.y;
    deltaX = goalCenterX - robot.x;

    bool x_first;
    int slope = deltaY / deltaX, remainder;
    // check if slope>1
    if (abs(deltaY) > abs(deltaX))
    {
        remainder = abs(deltaY) % abs(deltaX);
        slope = abs(deltaY) / abs(deltaX);
        x_first = true;
    }
    else
    {
        remainder = abs(deltaX) % abs(deltaY);
        slope = abs(deltaX) / abs(deltaY);
        x_first = false;
    }

    // main while loop
    while (true)
    {
        //++++++++++++++WRITE YOUR CODE HERE++++++++++++++++++++++++++++++++++

        // Task 1 (Check if robot hit boundary)
        if ((robot.x - radius) <= 0 || (robot.x + radius) >= width || (robot.y - radius) <= 0 || (robot.y + radius) >= height)
        {
            succeed = false;
            std::cout << "Robot hit boundary" << std::endl;
            break;
        }

        // Task 3 (1 Direction Pathfinding-X Movement First)
         int deltaY=goal.y-robot.y;
         int deltaX=goal.x-robot.x;
         std::cout<<((deltaX > 0) - (deltaX < 0))<<" ";
         if (deltaX!=0){
             robot.x+=1*((deltaX > 0) - (deltaX < 0));
         }else if (deltaY!=0){
             robot.y+=1*((deltaY > 0) - (deltaY < 0));
         }

        // Task 4 (1 Direction Pathfinding-Y Movement First)
        //  if (deltaY!=0){
        //      robot.y+=1*((deltaY > 0) - (deltaY < 0));
        //  }else if (deltaX!=0){
        //      robot.x+=1*((deltaX > 0) - (deltaX < 0));
        //  }

        // Task 5 (Diagonal Straight Line Pathfinding to top left corners)
        // if (x_first && deltaX != 0)
        // {
        //     robot.x += 1 * ((deltaX > 0) - (deltaX < 0));
        //     robot.y += slope * ((deltaY > 0) - (deltaY < 0));
        // }
        // if (x_first && deltaX == 0 && remainder != 0)
        // {
        //     robot.y += 1 * ((deltaY > 0) - (deltaY < 0));
        //     remainder -= 1;
        // }
        // if (!x_first && deltaY != 0)
        // {
        //     robot.y += 1 * ((deltaY > 0) - (deltaY < 0));
        //     robot.x += slope * ((deltaX > 0) - (deltaX < 0));
        // }
        // if (!x_first && deltaY == 0 && remainder != 0)
        // {
        //     robot.x += 1 * ((deltaX > 0) - (deltaX < 0));
        //     remainder -= 1;
        // }
        //Task 5
        // deltaY = goal.y - robot.y;
        // deltaX = goal.x - robot.x;
        
        //Task 6: To center of goal 
        deltaY = goalCenterY - robot.y;
        deltaX = goalCenterX - robot.x;

        // Task 2: Check if robot reached goal
        if (is_goal_detected(robot, goal))
        {
            succeed = true;
            std::cout << "Robot reached the goal!" << std::endl;
            break;
        }

        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //++++++++++++++++END YOUR CODE HERE++++++++++++++++++++++++++++++++++
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // Example provided: simply loop 3600 times and move the robot right each time.
        // This will also show a fail message as the succeed variable was never set to true
        // Do not change the while loop as it's made to end after 1 minute. This is to force this loop to eventually end so students can visualize code that gets stuck
        // You can define other functions to use outside of the main function if you wish
        // You may also define your own local variables inside main in addition to your own global variables. Make sure to know your variable scope

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