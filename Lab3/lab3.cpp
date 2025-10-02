#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>

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
            // arguments are (width, height, env_width, env_height)
        // define the grid that the robot maps. It can be a nested array or vector of size 800x800
            // example nested vector below
        std::vector<std::vector<int>> grid;
        // define the robot's sensor that defines (x, y) points of its grid as occupied/unoccupied/unknown (1/0/-1)
        // use Object's grid_value() function to get grid values
            // arguments are (grid_util, this, x query, y query, range)
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
const int min_obj_size {50};                        //Maximum object dimension. Not required for lab 3
const int max_obj_size {100};                       //Maximum object dimension. Not required for lab 3
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
    std::pair<std::string, bool> config = read_csv();

    // create the walls
    std::vector<Object*> walls;

    // normal perpendicular walls
    if (config.first == "environment1.csv") {
        walls = grid.create_walls(config.first);
    }
    // angled walls
    else {
        walls = grid.create_angled_walls(config.first);
    }

    // Uncomment this line to write the grid to csv to see the grid as a csv
    // grid.writeGridToCSV("grid.csv"); 

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++DEFINE ANY LOCAL VARIABLES HERE+++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++Modify the instantiation of robot++++++++++++++++
//++robot should be a my_robot class instead of an Object class++++++
//++++++++The constructor signature can be however you like++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // 2*radius is used for width/height of robot
    Object robot(2*radius, 2*radius, env_width, env_height);

    // create a copy. change this to a my_robot class as well
    Object robot_init = robot;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

        if (limit_count>=3600) {
            std::cout << "====Program terminated after 3600 iterations====" << std::endl;
            break;
        }
    }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++For now, an 800x800 vector, vec, initialized to -1 is placed here+++++++++
//+Modify line 175 so robot.grid is passed to grid.grid_accuracy() instead of vec+
//++++++Modify line 180 so the third argument to render_grid() is robot.grid++++++
//++++++After you make the robot instance in line 133 a my_robot class with 
//      a grid member, line 175 will have robot.grid as its argument
//      and line 180 will have robot.grid as its third argument.++++++++++++++++++
//++++++++++++++++++++++++++Then, you can remove vec++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    std::vector<std::vector<int>> vec(800, std::vector<int>(800, -1));
    float accuracy = grid.grid_accuracy(vec);
    std::cout << "Percent of walls correctly mapped: " << accuracy*100.0 << "%" << std::endl;
    if (config.second){
        render_window(robot_pos, walls, robot_init, env_width, env_height);
    }
    render_grid(robot_init, robot_pos, vec, env_width, env_height, radius, lidar_range);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    return 0;
}
