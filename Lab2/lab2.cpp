#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "utils.h"
#include "render.h"

//===== Main parameters =====
const int width {800}, height {800};        //Width and height of the environment
const int radius {10};                      //Radius of the robot's circular body
const int min_obj_size {50};                //Maximum object dimension
const int max_obj_size {100};               //Maximum object dimension
const int goal_width {100};                 //Goal width
const int goal_height {100};                //Goal heigth
const int robot_tol {50};                  //Tolerance for robot spawn point
const int occupancy_tol {30};               //Minimum distance between all objects that spawn
const int goal_tol {50};                   //Minimum distance in x,y between robot and goal
const int robot_y_min {500};                //Minimum robot y position
const int goal_y_max {300};                 //Maximum goal y position
int obj_x, obj_y, obj_width, obj_height;    //Parameters for object position/size
int num_objects {15};                       //Number of objects in environment

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

// e.g. void my_func() {}
// e.g. int a = 5;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char const *argv[])
{
    //==========CREATE ROBOT, GOAL, OBJECTS==========
    // create robot
    Object robot = grid.create_object(grid, rand_gen, robot_tol, 2*radius, 2*radius, robot_y_min, height-radius, 1, "robot");

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

    // main loop
    while (true)
    {
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++WRITE YOUR CODE HERE++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Example provided: simply loop 3600 times and move the robot right each time. 
        // This will also show a fail message as the succeed variable was never set to true
    // Do not change the while loop as it's made to end after 1 minute. This is to force this loop to eventually end so students can visualize code that gets stuck
    // You can define other functions to use outside of the main function if you wish
    // You may also define your own local variables inside main in addition to your own global variables. Make sure to know your variable scope

        robot.x += 1;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++END YOUR CODE HERE++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // place the current robot position at the time step to robot_pos
        robot_pos.push_back({robot.x, robot.y});
        max_count++;

        // if more than a minute passed (in render window), exit
        if (max_count>=3600) {
            std::cout << "=====1 minute reached with no solution=====" << std::endl;
            break;
        }
    }

    // send the results of the code to the renderer
    render_window(robot_pos, objects, robot_init, goal_init, width, height, succeed);
    return 0;
}
