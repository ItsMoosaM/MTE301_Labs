#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>  
#include <iostream>
#include <random>
#include <sstream>
#include <utility>
#include <vector>

#include "utils.h"
#include "render.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++Modify my_robot class here+++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// my_robot sub-class
// modify here so it inherits from the Object class from utils.h
class my_robot: public Object {
    // define any private or protected members here
    public:
        // constructor. modify accordingly
        my_robot(int width, int height, const int env_width, const int env_height, 
            int range, int tol, int min_y_spawn, int max_y_spawn)
            : Object(width, height, env_width, min_y_spawn, max_y_spawn, tol)
        {
            grid = std::vector<std::vector<int>>(env_height, std::vector<int>(env_width, -1));
        }

        // save grid
        void save_grid_csv() {
            std::string filename = "grid_pred.csv";
            std::ofstream file(filename);

            if (!file.is_open()) {
                std::cerr << "Error: Could not open file " << filename << std::endl;
                return;
            }

            // determine the maximum row size by finding the size of the longest inner vector
            size_t maxRowSize = 0;
            for (const auto& col : grid) {
                if (col.size() > maxRowSize) {
                    maxRowSize = col.size();
                }
            }

            // output the grid in transposed form (columns become rows in CSV)
            for (size_t row = 0; row < maxRowSize; ++row) {
                for (size_t col = 0; col < grid.size(); ++col) {
                    if (row < grid[col].size()) {
                        file << grid[col][row];
                    }
                    if (col < grid.size() - 1) {
                        file << ","; // Add comma except after the last element
                    }
                }
                file << "\n"; // New line after each row
            }

            file.close();
            std::cout << "Robot's grid written to " << filename << std::endl;
        }      

        // its known grid
        std::vector<std::vector<int>> grid;
        // keep the same sensor developed in lab 3/4
        // keep the same wall following from lab 4.
        // develop obstacle avoidance with the sweep algorithm
        // develop goal navigation with obstacle avoidance
        // define any other public members and functions you wish to use

};

//===== Main parameters =====
const int env_width {800}, env_height {800};    //Width and height of the environment
const int radius {10};                          //Radius of the robot's circular body
const int min_obj_size {30};                    //Maximum object dimension
const int max_obj_size {40};                    //Maximum object dimension
const int occupancy_tol {35};                   //Minimum distance between all objects that spawn
int lidar_range{40};                            //Lidar range, radiating from center of robot
int tol{5+radius};                              //How much closer from farthest lidar range should robot stop in front of obstacle?
int num_objects {6};                            //Number of objects in environment

// Grid utility class
grid_util grid(env_width, env_height, min_obj_size, max_obj_size, radius, tol);

// Random generator
random_generator rand_gen;

// Vector of robot positions
std::vector<std::vector<int>> robot_pos;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++DEFINE ANY GLOBAL VARIABLES/FUNCTIONS HERE+++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char const *argv[])
{

    //==========CREATE ROBOT, GOAL, OBJECTS==========

    // read config file
    std::tuple<std::string, bool, int, int> config = read_csv();

    // create the walls
    std::vector<Object*> objects;
    
    // normal perpendicular walls
    if (std::get<3>(config) == 4) {
        objects = grid.create_walls(std::get<0>(config));
    }
    // angled walls
    else {
        objects = grid.create_angled_walls(std::get<0>(config));
    }

    // create the goal
    Object* goal = grid.spawn_object(rand_gen, occupancy_tol, 2);
    goal->val = 2;

    int min_y_spawn = grid.get_min_y();
    int max_y_spawn = grid.get_max_y();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // create the objects
    std::vector<Object *> obstacles = grid.create_objects_wall(rand_gen, occupancy_tol, tol, radius, num_objects);

    // push obstacles to the list of objects
    objects.insert(objects.end(), obstacles.begin(), obstacles.end());
    // insert goal to list of objects
    objects.push_back(goal);

    // clear the grid of -1 tolerance values
    grid.clear_tol();

    // Uncomment this line to write the grid to csv to see the grid as a csv
    grid.writeGridToCSV("grid.csv");

    // create robot with range sensor of range 40
    my_robot robot(2*radius, 2*radius, env_width, env_height, 
        lidar_range, tol, min_y_spawn, max_y_spawn);

    // create a copy
    my_robot robot_init = robot;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++DEFINE ANY LOCAL VARIABLES HERE+++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    robot_pos.push_back({robot.x, robot.y});
    int limit_count = 0;

    // run the program indefinitely until robot hits the goal or an obstacle
    while (true)
    {
        limit_count++;

//+++++++++++++++WRITE YOUR MAIN LOOP CODE HERE++++++++++++++++++++++
//++++++++++++++EXAMPLE: ROBOT SIMPLY MOVES LEFT+++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        robot.x -= 1;      

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  
            
//++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // update position
        robot_pos.push_back({robot.x, robot.y});

        if (limit_count>=10000) {
            std::cout << "====Program terminated after " << limit_count << " iterations====" << std::endl;
            break;
        }

    }

    float wall_accuracy = grid.wall_accuracy(robot.grid);   // for task 1: outer walls
    float accuracy = grid.grid_accuracy(robot.grid);        // for task 2: entire environment inside walls
    std::cout << std::fixed << std::setprecision(2);        // set precision for printing
    std::cout << "Percent of walls correctly mapped: " << wall_accuracy*100.0 << "%" << std::endl;
    std::cout << "Percent of environment correctly mapped: " << accuracy*100.0 << "%" << std::endl;
    if (std::get<1>(config)){
        render_window(robot_pos, objects, robot_init, env_width, env_height, std::get<2>(config));
    }
    render_grid(robot_init, robot_pos, robot.grid, env_width, env_height, radius, lidar_range, std::get<2>(config));
    return 0;
}
