#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <fstream>
#include <utility>
#include <iomanip>
#include <queue>

#include "utils.h"
#include "render.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++Modify my_robot class here+++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// my_robot sub-class
class my_robot : public Object
{
    // define any private or protected members here
private:
    std::vector<int> prev_mode{0, 0, 0, 0};
    std::queue<std::pair<int, int>> paths_queue;

public:
    // define constructor with necessary parameters
    int lidar_range = 50;
    std::vector<std::vector<int>> grid; // robot’s local occupancy grid
    int tol_45 = 0;
    // call the constructor of the Object base class
    // arguments are (width, height, env_width, env_height)
    // define the grid that the robot maps. It can be a nested array or vector of size 800x800
    my_robot(int width, int height, int env_width, int min_y, int max_y, int tol)
        : Object(width, height, env_width, min_y, max_y, tol)
    {
        // lidar_range = 50;
        tol_45 = static_cast<int>(sin(M_PI / 4) * tol);
        grid = std::vector<std::vector<int>>(800, std::vector<int>(800, -1));
    }
    // Lab 2 Part 1 Task 1 Sensor Function
    //  define the robot's sensor that defines (x, y) points of its grid as occupied/unoccupied/unknown (1/0/-1)
    void robotSensor(grid_util &true_grid)
    {
        int radius = this->width / 2;
        int x_c = this->x + radius, y_c = this->y + radius;

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

    // Lab 2 Part 2 Task 1 Modified Wall Functions
    std::vector<int> wallDetection()
    {
        // x = (1/0/-1) if (right hit/free/left hit)
        // y = (1/0/-1) if (bottom hit/free/top hit)
        // v = (1/0/-1) if (bottom right hit/free/top left hit)
        // w = (1/0/-1) if (top right hit/free/bottom left hit)
        std::vector<int> mode{0, 0, 0, 0};

        int radius = this->width / 2;
        int c_x = this->x + radius, c_y = this->y + radius;
        int &x = mode[0], &y = mode[1], &v = mode[2], &w = mode[3];
        int tol = radius + 5; // Center of robot from wall

        if (grid[c_x + tol][c_y] == 1)
        {
            x = 1;
        }
        else if (grid[c_x - tol][c_y] == 1)
        {
            x = -1;
        }
        // else
        // {
        //     x = 0;
        // }
        if (grid[c_x][c_y + tol] == 1)
        {
            y = 1;
        }
        else if (grid[c_x][c_y - tol] == 1)
        {
            y = -1;
        }
        // else
        // {
        //     y = 0;
        // }
        // Check v/w
        if (grid[c_x + tol_45][c_y + tol_45] == 1) // bottom right
        {
            v = 1;
        }
        else if (grid[c_x - tol_45][c_y - tol_45] == 1) // top left
        {
            v = -1;
        }
        // else
        // {
        //     v = 0;
        // }
        if (grid[c_x + tol_45][c_y - tol_45] == 1) // top right
        {
            w = 1;
        }
        else if (grid[c_x - tol_45][c_y + tol_45] == 1) // bottom left
        {
            w = -1;
        }
        // else
        // {
        //     w = 0;
        // }
        return mode;
    }

    std::pair<int, int> findDirection(std::vector<int> mode)
    {
        // auto mode = modeArray;
        int xw = 0, yw = 1;

        if (mode != prev_mode)
        {
            while (!paths_queue.empty())
            {
                paths_queue.pop();
            }

            // FIXED: Improved logic with better corner handling
            // Check for multiple wall collisions first
            if (mode[0] == 1 && mode[1] == 1)
            { // Right-Bottom corner
                xw = -1;
                yw = 1; // Move down-left
            }
            else if (mode[0] == 1 && mode[1] == -1)
            { // Right-Top corner
                xw = 1;
                yw = -1; // Move up-right
            }
            else if (mode[0] == -1 && mode[1] == 1)
            { // Left-Bottom corner
                xw = -1;
                yw = 1; // Move down-left
            }
            else if (mode[0] == -1 && mode[1] == -1)
            { // Left-Top corner
                xw = -1;
                yw = -1; // Move up-left
            }
            // Single walls
            else if (mode[0] == 1)
            { // Right wall
                xw = 0;
                yw = 1; // Move down
            }
            else if (mode[0] == -1)
            { // Left wall
                xw = 0;
                yw = -1; // Move up
            }
            else if (mode[1] == 1)
            { // Bottom wall
                xw = -1;
                yw = 0; // Move left
            }
            else if (mode[1] == -1)
            { // Top wall
                xw = 1;
                yw = 0; // Move right
            }
            // Diagonals
            else if (mode[2] == 1)
            { // Bottom-right
                xw = 1;
                yw = 1; // Move down-right
            }
            else if (mode[2] == -1)
            { // Top-left
                xw = -1;
                yw = -1; // Move up-left
            }
            else if (mode[3] == 1)
            { // Top-right
                xw = 1;
                yw = -1; // Move up-right
            }
            else if (mode[3] == -1)
            { // Bottom-left
                xw = -1;
                yw = 1; // Move down-left
            }
            else
            {
                // No walls - move to find walls
                xw = 1;
                yw = 0; // Move right
            }

            prev_mode = mode;
        }

        return std::make_pair(xw, yw);
    }
    void move(std::pair<int, int> robot_to_wall)
    {
        // define direction to travel(x_dir, y_dir)
        int x_dir = robot_to_wall.first;
        int y_dir = robot_to_wall.second;

        bool clockwise = true;

        // --- Lines 1–6 of pseudocode ---
        if (clockwise)
        {
            // Clockwise: x_dir = -abs(robot_to_wall.second)/robot_to_wall.second
            //           y_dir = abs(robot_to_wall.first)/robot_to_wall.first
            if (robot_to_wall.second != 0)
            {
                x_dir = -abs(robot_to_wall.second) / robot_to_wall.second;
            }
            if (robot_to_wall.first != 0)
            {
                y_dir = abs(robot_to_wall.first) / robot_to_wall.first;
            }

            // FIXED: Simplified movement without complex calculations
            // this->x += x_dir;
            // this->y += y_dir;
            // this->x = std::max(0, std::min(799, this->x));
            // this->y = std::max(0, std::min(799, this->y));
            // //  x_dir = robot_to_wall.first;
            // //  y_dir = robot_to_wall.second;
        }
        else
        {
            // Counterclockwise: x_dir = abs(robot_to_wall.second)/robot_to_wall.second
            //                  y_dir = -abs(robot_to_wall.first)/robot_to_wall.first
            if (robot_to_wall.second != 0)
            {
                x_dir = abs(robot_to_wall.second) / robot_to_wall.second;
            }
            if (robot_to_wall.first != 0)
            {
                y_dir = -abs(robot_to_wall.first) / robot_to_wall.first;
            }
        }

        // Lines 7-13: Path queue implementation
        const int n = 5;

        if (paths_queue.empty())
        {
            for (int i = 1; i <= n; i++)
            {
                paths_queue.push({this->x + x_dir * i, this->y + y_dir * i});
            }
        }

        if (!paths_queue.empty())
        {
            auto next_pos = paths_queue.front();
            this->x = next_pos.first;
            this->y = next_pos.second;
            paths_queue.pop();
        }
        return;
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
const int min_obj_size{50};                // Maximum object dimension. Not required for lab 3/4
const int max_obj_size{100};               // Maximum object dimension. Not required for lab 3/4
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
    std::tuple<std::string, bool, int, int> config = read_csv();

    // create the walls
    std::vector<Object *> walls;

    // normal perpendicular walls
    if (std::get<3>(config) == 4)
    {
        walls = grid.create_walls(std::get<0>(config));
    }
    // angled walls
    else
    {
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
    my_robot robot(2 * radius, 2 * radius, env_width, min_y_spawn, max_y_spawn, 15);
    // create a copy. change this to a my_robot class as well
    my_robot robot_init = robot;

    // push the initial position onto robot_pos
    robot_pos.push_back({robot.x, robot.y});
    int limit_count = 0;

    // run the program indefinitely until robot hits the goal or an obstacle
    while (true)
    {
        limit_count++;
        //+++++++++++++++WRITE YOUR MAIN LOOP CODE HERE++++++++++++++++++++++
        robot.robotSensor(grid);

        auto mode = robot.wallDetection();
        auto robot_to_wall = robot.findDirection(mode);
        robot.move(robot_to_wall);

        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        robot_pos.push_back({robot.x, robot.y});

        if (limit_count >= 7200)
        {
            std::cout << "====Program terminated after 7200 iterations====" << std::endl;
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
    std::cout << std::fixed << std::setprecision(2);      // set precision for printing
    float wall_accuracy = grid.wall_accuracy(robot.grid); // for task 1: outer walls. replace vec with your robot's grid
    float accuracy = grid.grid_accuracy(robot.grid);      // for task 2: entire environment inside walls. replace vec with your robot's grid
    std::cout << "Percent of walls correctly mapped: " << wall_accuracy * 100.0 << "%" << std::endl;
    std::cout << "Percent of environment correctly mapped: " << accuracy * 100.0 << "%" << std::endl;
    if (std::get<1>(config))
    {
        render_window(robot_pos, walls, robot_init, env_width, env_height, std::get<2>(config));
    }
    render_grid(robot_init, robot_pos, robot.grid, env_width, env_height, radius, lidar_range, std::get<2>(config));
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    return 0;
}
