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
class my_robot : public Object
{
    // define any private or protected members here
public:
    // define constructor with necessary parameters
    int lidar_range = 50;
    std::vector<std::vector<int>> grid; // robot’s local occupancy grid

    // call the constructor of the Object base class
    // arguments are (width, height, env_width, env_height)
    // define the grid that the robot maps. It can be a nested array or vector of size 800x800
    my_robot(int width, int height, int env_width, int env_height)
        : Object(width, height, env_width, env_height)
    {
        grid = std::vector<std::vector<int>>(800, std::vector<int>(800, -1));
    }
    // Task 1 Sensor Function
    //  define the robot's sensor that defines (x, y) points of its grid as occupied/unoccupied/unknown (1/0/-1)
    void robotSensor(grid_util &true_grid)
    {
        int radius = this->width / 2;
        int x_c = this->x + radius, y_c = this->y + radius, lidar_range = 50;

        for (int i = x_c - lidar_range; i <= x_c + lidar_range; i++)
        {
            for (int j = y_c - lidar_range; j <= y_c + lidar_range; j++)
            {
                if (i < 0 || j < 0 || i >= 800 || j >= 800)
                    continue; // skip out-of-bounds
                if ((i - x_c) * (i - x_c) + (j - y_c) * (j - y_c) <= lidar_range * lidar_range)
                {
                    grid[i][j] = Object::grid_value(true_grid, this, i, j, lidar_range);
                }
            }
        }
    }
    // use Object's grid_value() function to get grid values
    // arguments are (grid_util, this, x query, y query, range)
    // define any other public members and functions you wish to use

    // Task 2 Wall Funcitons
    std::pair<int, int> collisionDetection()
    {
        // v = (1/0/-1) if (top hit/free/bottom hit)
        // h = (1/0/-1) if (right hit/free/left hit)
        int radius = this->width / 2;
        int v = 0, h = 0, c_x = this->x + radius, c_y = this->y + radius;
        int tol = 15; // Center of robot from wall

        if (grid[c_x][c_y - tol] == 1)
        {
            v = 1;
        }
        else if (grid[c_x][c_y + tol] == 1)
        {
            v = -1;
        }
        else
        {
            v = 0;
        }
        if (grid[c_x + tol][c_y] == 1)
        {
            h = 1;
        }
        else if (grid[c_x - tol][c_y] == 1)
        {
            h = -1;
        }
        else
        {
            h = 0;
        }

        return std::make_pair(v, h);
    }
    void move(int v, int h)
    {
        int speed = 1;
        if (h == 1 && v == 1)
        {
            this->y += speed;
        }
        if (h == 1 && v == 0)
        {
            this->y += speed;
        }
        if (h == 1 && v == -1)
        {
            this->x -= speed;
        }
        if (h == 0 && v == 1)
        {
            this->x += speed;
        }
        if (h == 0 && v == 0)
        {
            this->x -= speed;
        }
        if (h == 0 && v == -1)
        {
            this->x -= speed;
        }
        if (h == -1 && v == 1)
        {
            this->x += speed;
        }
        if (h == -1 && v == 0)
        {
            this->y -= speed;
        }
        if (h == -1 && v == -1)
        {
            this->y -= speed;
        }
    }
    // function to save predicted grid
    void save_grid_csv()
    {
        std::string filename = "grid_pred.csv";
        std::ofstream file(filename);

        if (!file.is_open())
        {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }

        // determine the maximum row size by finding the size of the longest inner vector
        size_t maxRowSize = 0;
        for (const auto &col : grid)
        {
            if (col.size() > maxRowSize)
            {
                maxRowSize = col.size();
            }
        }

        // output the grid in transposed form (columns become rows in CSV)
        for (size_t row = 0; row < maxRowSize; ++row)
        {
            for (size_t col = 0; col < grid.size(); ++col)
            {
                if (row < grid[col].size())
                {
                    file << grid[col][row];
                }
                if (col < grid.size() - 1)
                {
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
const int env_width{800}, env_height{800}; // Width and height of the environment
const int radius{10};                      // Radius of the robot's circular body
const int min_obj_size{50};                // Maximum object dimension. Not required for lab 3
const int max_obj_size{100};               // Maximum object dimension. Not required for lab 3
int lidar_range{50};                       // Lidar range, radiating from center of robot

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
    std::vector<Object *> walls;

    // normal perpendicular walls
    if (config.first == "environment1.csv")
    {
        walls = grid.create_walls(config.first);
    }
    // angled walls
    else
    {
        walls = grid.create_angled_walls(config.first);
    }

    // Uncomment this line to write the grid to csv to see the grid as a csv
    // grid.writeGridToCSV("grid.csv");

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+++++++++++++++DEFINE ANY LOCAL VARIABLES HERE+++++++++++++++++++++
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //++++++++++++++++++Modify the instantiation of robot++++++++++++++++
    //++robot should be a my_robot class instead of an Object class++++++
    //++++++++The constructor signature can be however you like++++++++++
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // 2*radius is used for width/height of robot
    my_robot robot(2 * radius, 2 * radius, env_width, env_height);
    // create a copy. change this to a my_robot class as well
    my_robot robot_init = robot;

    // push the initial position onto robot_pos
    robot_pos.push_back({robot.x, robot.y});
    int limit_count = 0;
    // run the program indefinitely until robot hits the goal or an obstacle
    while (true)
    {
        limit_count++;

        robot.robotSensor(grid);

        auto collisionVals = robot.collisionDetection();
        robot.move(collisionVals.first, collisionVals.second);
        // robot.x -= 1;
        robot_pos.push_back({robot.x, robot.y});

        if (limit_count >= 3600)
        {
            std::cout << "====Program terminated after 3600 iterations====" << std::endl;
            break;
        }
    }

    float accuracy = grid.grid_accuracy(robot.grid);
    std::cout << "Percent of walls correctly mapped: " << accuracy * 100.0 << "%" << std::endl;
    if (config.second)
    {
        render_window(robot_pos, walls, robot_init, env_width, env_height);
    }
    render_grid(robot_init, robot_pos, robot.grid, env_width, env_height, radius, lidar_range);
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    return 0;
}
