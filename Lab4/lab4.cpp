#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>
#include <iomanip>

#include "utils.h"
#include "render.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++Modify my_robot class here+++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// my_robot sub-class
// modify here so it inherits from the Object class from utils.h
class my_robot {
    // define any private or protected members here
    public:
        // define constructor with necessary parameters
            // example constructor that simply initializes grid to all -1
        my_robot() {
            grid = std::vector<std::vector<int>>(800, std::vector<int>(800, -1));
        }
        // call the constructor of the Object base class
            // arguments are (width, height, env_width, min_y, max_y, tol)
        // define the grid that the robot maps. It can be a nested array or vector of size 800x800
            // example nested vector below
        std::vector<std::vector<int>> grid;
        // keep the same sensor developed in lab 3
        // develop the wall following/sweep algorithms
        // define any other public members and functions you wish to use

        // function to save predicted grid
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
};
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//===== Main parameters =====
const int env_width {800}, env_height {800};        //Width and height of the environment
const int radius {10};                              //Radius of the robot's circular body
const int min_obj_size {50};                        //Maximum object dimension. Not required for lab 3/4
const int max_obj_size {100};                       //Maximum object dimension. Not required for lab 3/4
int lidar_range{50};                                //Lidar range, radiating from center of robot

// Grid utility class
grid_util grid(env_width, env_height, min_obj_size, max_obj_size);

// Random generator
random_generator rand_gen;

// Vector of velocity commands
std::vector<std::vector<int>> robot_pos;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++DEFINE ANY GLOBAL VARIABLES/FUNCTIONS HERE+++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char const *argv[])
{
    //==========CREATE ROBOT AND WALLS==========

    // read config file
    std::tuple<std::string, bool, int, int> config = read_csv();

    // create the walls
    std::vector<Object*> walls;

    // normal perpendicular walls
    if (std::get<3>(config) == 4) {
        walls = grid.create_walls(std::get<0>(config));
    }
    // angled walls
    else {
        walls = grid.create_angled_walls(std::get<0>(config));
    }

    // get minimum/maximum y values for the robot to spawn
    int min_y_spawn = grid.get_min_y();
    int max_y_spawn = grid.get_max_y();

    // Uncomment this line to write the grid to csv to see the grid as a csv
    // grid.writeGridToCSV("grid.csv"); 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++DEFINE ANY LOCAL VARIABLES HERE+++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++Modify the instantiation of robot+++++++++++++++++++++++++++
//+++++robot should be a my_robot class instead of an Object class++++++++++++++++
//+++++++++++The constructor signature can be however you like++++++++++++++++++++
//+++Make sure to pass min_y_spawn and max_y_spawn to the constructor of Object+++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // 2*radius is used for width/height of robot
    Object robot(2*radius, 2*radius, env_width, min_y_spawn, max_y_spawn, radius+5);

    // create a copy. change this to a my_robot class as well
    Object robot_init = robot;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // push the initial position onto robot_pos
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
        robot_pos.push_back({robot.x, robot.y});

        if (limit_count>=7200) {
            std::cout << "====Program terminated after 3600 iterations====" << std::endl;
            break;
        }
    }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++For now, an 800x800 vector, vec, initialized to -1 is placed here+++++++++
//+Modify lines 180-181 so robot.grid is passed to both functions instead of vec++
//++++++Modify line 187 so the third argument to render_grid() is robot.grid++++++
//++++++After you make the robot instance in line 139 a my_robot class with 
//      a grid member, lines 182-183 will have robot.grid as its argument
//      and line 189 will have robot.grid as its third argument.++++++++++++++++++
//++++++++++++++++++++++++++Then, you can remove vec++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    std::vector<std::vector<int>> vec(800, std::vector<int>(800, -1));
    std::cout << std::fixed << std::setprecision(2);        // set precision for printing
    float wall_accuracy = grid.wall_accuracy(vec);          // for task 1: outer walls. replace vec with your robot's grid
    float accuracy = grid.grid_accuracy(vec);               // for task 2: entire environment inside walls. replace vec with your robot's grid
    std::cout << "Percent of walls correctly mapped: " << wall_accuracy*100.0 << "%" << std::endl;
    std::cout << "Percent of environment correctly mapped: " << accuracy*100.0 << "%" << std::endl;
    if (std::get<1>(config)){
        render_window(robot_pos, walls, robot_init, env_width, env_height, std::get<2>(config));
    }
    render_grid(robot_init, robot_pos, vec, env_width, env_height, radius, lidar_range, std::get<2>(config));
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    return 0;
}
