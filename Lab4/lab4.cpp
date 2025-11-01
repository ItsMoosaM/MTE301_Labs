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

using IntVector = std::vector<int>;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++Modify my_robot class here+++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// my_robot sub-class
class my_robot : public Object
{
    // define any private or protected members here
private:
public:
    // define constructor with necessary parameters
    int lidar_range = 50;
    std::vector<std::vector<int>> grid; // robot’s local occupancy grid
    int tol = 0, tol_45 = 0;
    bool clockwise = true;

    // call the constructor of the Object base class
    // arguments are (width, height, env_width, env_height)
    // define the grid that the robot maps. It can be a nested array or vector of size 800x800
    my_robot(int width, int height, int env_width, int min_y, int max_y, int tole)
        : Object(width, height, env_width, min_y, max_y, tole)
    {
        // lidar_range = 50;
        tol = tole;
        tol_45 = static_cast<int>(sin(M_PI / 4) * tol);
        std::cout << tol_45;
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
    std::vector<int> detect_walls()
    {
        // x = (1/0/-1) if (right hit/free/left hit)
        // y = (1/0/-1) if (bottom hit/free/top hit)
        // v = (1/0/-1) if (bottom right hit/free/top left hit)
        // w = (1/0/-1) if (top right hit/free/bottom left hit)
        std::vector<int> mode{0, 0, 0, 0};

        int radius = this->width / 2;
        int c_x = this->x + radius, c_y = this->y + radius;
        int &x = mode[0], &y = mode[1], &v = mode[2], &w = mode[3];
        // int tol = radius + 5; // Center of robot from wall

        // left/right
        if (grid[c_x + tol][c_y] == 1)
        {
            x = 1;
        }
        else if (grid[c_x - tol][c_y] == 1)
        {
            x = -1;
        }
        else
        {
            x = 0;
        }
        if (grid[c_x][c_y + tol] == 1)
        {
            y = 1;
        }
        else if (grid[c_x][c_y - tol] == 1)
        {
            y = -1;
        }
        else
        {
            y = 0;
        }
        // Check v/w
        if (grid[c_x + tol_45][c_y + tol_45] == 1) // bottom right
        {
            v = 1;
        }
        else if (grid[c_x - tol_45][c_y - tol_45] == 1) // top left
        {
            v = -1;
        }
        else
        {
            v = 0;
        }
        if (grid[c_x + tol_45][c_y - tol_45] == 1) // top right
        {
            w = 1;
        }
        else if (grid[c_x - tol_45][c_y + tol_45] == 1) // bottom left
        {
            w = -1;
        }
        else
        {
            w = 0;
        }

        return mode;
    }

    std::vector<int> prev_mode{0, 0, 0, 0};
    std::queue<std::pair<int, int>> paths_queue;

    std::pair<int, int> find_dir(std::vector<int> mode)
    {
        auto currentMode = mode;

        // wall vectors
        int x_wall = 0, y_wall = 0;

        // check hits:
        int curr_hits = 0, prev_hits = 0;
        for (int val : currentMode)
            if (val != 0)
                curr_hits++;
        for (int val : prev_mode)
            if (val != 0)
                prev_hits++;

        if (curr_hits == 0)
        {
            y_wall = 1;
        }
        else if (prev_hits <= 1 && curr_hits >= 1)
        {
            for (int i = 0; i < 4; i++)
            {
                mode[i] = currentMode[i] - prev_mode[i];
            }
        }
        else if (prev_hits >= 2 && curr_hits >= 1)
        {
            mode = currentMode;
        }
        // X
        if (mode == IntVector{1, 0, 0, 0} || mode == IntVector{-1, 0, 0, 0})
        {
            x_wall = mode[0];
        }
        // Y
        else if (mode == IntVector{0, 1, 0, 0} || mode == IntVector{0, -1, 0, 0})
        {
            y_wall = mode[1];
        }
        // V
        else if (mode == IntVector{0, 0, 1, 0} || mode == IntVector{0, 0, -1, 0})
        {
            x_wall = mode[2] * tol_45;
            y_wall = mode[2] * tol_45;
        }
        // W
        else if (mode == IntVector{0, 0, 0, 1} || mode == IntVector{0, 0, 0, -1})
        {
            x_wall = mode[3] * tol_45;
            y_wall = -1 * mode[3] * tol_45;
        }

        while (!paths_queue.empty())
        {
            paths_queue.pop();
        }
        prev_mode = mode;

        // Return clockwise dir vector
        return std::make_pair(x_wall, y_wall);
    }
    void move(std::pair<int, int> robot_to_wall)
    {
        // define direction to travel(x_dir, y_dir)
        int x_dir = 0;
        int y_dir = 0;

        if (clockwise)
        {
            // Clockwise: x_dir = -abs(robot_to_wall.second)/robot_to_wall.second
            //           y_dir = abs(robot_to_wall.first)/robot_to_wall.first
            if (robot_to_wall.second != 0)
            {
                x_dir = -1 * std::abs(robot_to_wall.second) / robot_to_wall.second;
            }
            if (robot_to_wall.first != 0)
            {
                y_dir = std::abs(robot_to_wall.first) / robot_to_wall.first;
            }
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
                y_dir = -1 * abs(robot_to_wall.first) / robot_to_wall.first;
            }
        }

        // Path queue
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

    bool sweep_mode = false;
    bool lapped = false;
    bool hitWall = false;
    int x_pos = 0, y_pos = 0;
    int y_ref = 0;
    int max_y = robot.y;
    int min_y = robot.y;
    // run the program indefinitely until robot hits the goal or an obstacle
    while (true)
    {
        limit_count++;
        //+++++++++++++++WRITE YOUR MAIN LOOP CODE HERE++++++++++++++++++++++
        robot.robotSensor(grid);
        auto mode = robot.detect_walls();

        // auto robot_to_wall = robot.find_dir(mode);
        // robot.move(robot_to_wall);
        // Lab 2 Part 2 Task 2
        if (!sweep_mode)
        {
            if (!hitWall && mode != IntVector{0, 0, 0, 0})
            {
                hitWall = true;
                x_pos = robot.x;
                y_pos = robot.y;
                std::cout << x_pos << " " << y_pos;
            }
            auto robot_to_wall = robot.find_dir(mode);
            robot.move(robot_to_wall);

            // Step 6: Update max/min Y boundaries
            if (robot.y > max_y)
            {
                max_y = robot.y;
            }
            if (robot.y < min_y)
            {
                min_y = robot.y;
            }
            // Step 7–9: Check for lapping condition (reaching max_y again)
            if (robot.y == y_pos && robot.x == x_pos)
            {
                lapped = true;
                // std::cout << "=== Entering SWEEP MODE at y_ref = " << y_ref << " ===" << std::endl;
            }
            if (robot.y <= min_y && lapped)
            {
                y_ref = robot.y;
                sweep_mode = true;
            }
        }
        else
        {
            auto robot_to_wall = robot.find_dir(mode);

            if (robot.prev_mode == IntVector{0, 0, 0, 0} && mode != IntVector{0, 0, 0, 0})
            {
                // hitWall = true;
                // robot.prev_mode==mode;
                // robot.clockwise = !robot.clockwise;
                // std::cout << "Toggled direction! Now " << (robot.clockwise ? "CW" : "CCW") << std::endl;
            }

            // Move normally along walls
            robot.move(robot_to_wall);

            // Move up one strip and shift left/right after y_ref + 50
            if (robot.y == y_ref + 50)
            {
                // std::vector<int> mode_side = mode;
                while (mode != IntVector{0, 0, 0, 0})
                {
                    // robot.move(robot_to_wall);
                    if (robot.clockwise)
                        robot.x -= 1; // move left
                    else
                        robot.x += 1; // move right
                    robot.robotSensor(grid);
                    mode = robot.detect_walls();
                }
                // Set new reference line
                y_ref = robot.y;
                std::cout << "Shifted to new sweep line at y=" << y_ref << std::endl;
            }
            // Stop after reaching bottom
            if (robot.y >= max_y - 5)
            {
                std::cout << "Reached max_y boundary. Sweep complete!" << std::endl;
                break;
            }
        }

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
