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
                if (i >= 0 && i < 800 && j >= 0 && j < 800)
                {
                    if ((i - x_c) * (i - x_c) + (j - y_c) * (j - y_c) <= lidar_range * lidar_range)
                    {
                        grid[i][j] = Object::grid_value(true_grid, this, i, j, lidar_range);
                    }
                }
            }
        }
    }
    // use Object's grid_value() function to get grid values
    // arguments are (grid_util, this, x query, y query, range)
    // define any other public members and functions you wish to use

    // Lab 2 Part 2 Task 1 Modified Wall Functions
    // following functions for task1
    std::vector<int> detect_walls()
    {
        std::vector<int> direction(4, 0);
        int radius = this->width / 2;
        int cx = this->x + radius;
        int cy = this->y + radius;
        int tol = radius + 10;
        const int tol_45 = tol * cos(45 * M_PI / 180);
        // Skip Out Of Bound Checks
        auto safe_grid_check = [&](int x, int y) -> int
        {
            if (x >= 0 && x < 800 && y >= 0 && y < 800)
            {
                return grid[x][y];
            }
            return -1; // Out of bounds treated as unknown
        };
        if (safe_grid_check(cx, cy - tol) == 1)
        {
            direction[0] = 1; // Top Wall
        }
        else if (safe_grid_check(cx, cy + tol) == 1)
        {
            direction[0] = -1; // Bottom wall
        }
        if (safe_grid_check(cx + tol, cy) == 1)
        {
            direction[1] = 1; // Right wall
        }
        else if (safe_grid_check(cx - tol, cy) == 1)
        {
            direction[1] = -1; // Left wall
        }
        if (safe_grid_check(cx - tol_45, cy - tol_45) == 1)
        {
            direction[2] = 1; // Top-left diagonal
        }
        else if (safe_grid_check(cx + tol_45, cy + tol_45) == 1)
        {
            direction[2] = -1; // Bottom-right diagonal
        }
        if (safe_grid_check(cx + tol_45, cy - tol_45) == 1)
        {
            direction[3] = 1; // Top-right diagonal
        }
        else if (safe_grid_check(cx - tol_45, cy + tol_45) == 1)
        {
            direction[3] = -1; // Bottom-left diagonal
        }
        return direction;
    }

    std::queue<std::vector<int>> paths_queue;
    void find_dir(const std::vector<int> &direction)
    {
        std::vector<int> movementVector(2, 0);

        // Give Default Movement to left
        if (direction[0] == 0 && direction[1] == 0 && direction[2] == 0 && direction[3] == 0)
        {
            movementVector = {-1, 0};
        }
        else
        {
            if (clockwise)
            {
                if (direction[2] == 1)
                {
                    movementVector = {1, -1};
                }
                if (direction[0] == 1 && direction[2] == 0 && direction[3] == 0)
                {
                    movementVector = {1, 0};
                }
                if (direction[3] == 1)
                {
                    movementVector = {1, 1};
                }
                if (direction[1] == 1 && direction[2] == 0 && direction[3] == 0)
                {
                    movementVector = {0, 1};
                }
                if (direction[2] == -1)
                {
                    movementVector = {-1, 1};
                }
                if (direction[0] == -1 && direction[2] == 0 && direction[3] == 0)
                {
                    movementVector = {-1, 0};
                }
                if (direction[3] == -1 && direction[2] != 1)
                {
                    movementVector = {-1, -1};
                }
                if (direction[1] == -1 && direction[2] == 0 && direction[3] == 0)
                {
                    movementVector = {0, -1};
                }
            }
            else
            {
                if (direction[2] == 1)
                {
                    movementVector = {-1, 1};
                }
                if (direction[1] == -1 && direction[2] == 0 && direction[3] == 0)
                {
                    movementVector = {0, 1};
                }
                if (direction[3] == -1)
                {
                    movementVector = {1, 1};
                }
                if (direction[0] == -1 && direction[2] == 0 && direction[3] == 0)
                {
                    movementVector = {1, 0};
                }
                if (direction[2] == -1)
                {
                    movementVector = {1, -1};
                }
                if (direction[1] == 1 && direction[2] == 0 && direction[3] == 0)
                {
                    movementVector = {0, -1};
                }
                if (direction[3] == 1 && direction[2] != 1)
                {
                    movementVector = {-1, -1};
                }
                if (direction[0] == 1 && direction[2] == 0 && direction[3] == 0)
                {
                    movementVector = {-1, 0};
                }
            }
        }
        //Add vector to queue
        paths_queue.push(movementVector);
    }
    //Move the robot using the queue
    void move()
    {
        if (!paths_queue.empty())
        {
            std::vector<int> moveVector = paths_queue.front();
            paths_queue.pop();

            // Update position
            this->x += moveVector[0];
            this->y += moveVector[1];
        }
    }

    void switch_dir()
    {
        clockwise = !clockwise;
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
using IntVector = std::vector<int>;
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

    //Sweeping Variables
    bool sweep_mode = false;
    int y_ref = 0, max_y = 0, min_y = 800;
    IntVector prev_direction(4, 0);
    const IntVector freeDirection(4, 0);
    // run the program indefinitely until robot hits the goal or an obstacle
    while (true)
    {
        limit_count++;
        //+++++++++++++++WRITE YOUR MAIN LOOP CODE HERE++++++++++++++++++++++
        robot.robotSensor(grid);

        // Lab 2 Part 2 Task 2
        if (!sweep_mode)
        {
            robot.find_dir(robot.detect_walls());
            robot.move();

            // Update max/min Y boundaries
            if (robot.y > max_y)
            {
                max_y = robot.y;
            }
            if (robot.y < min_y)
            {
                min_y = robot.y;
            }
            // make sure at min_y and walls are mapped
            if (grid.wall_accuracy(robot.grid) >= .95 && robot.y == min_y)
            {
                y_ref = robot.y;
                robot.switch_dir();
                sweep_mode = true;
            }
        }
        else
        {
            robot.find_dir(robot.detect_walls());
            if (prev_direction == freeDirection && robot.detect_walls() != freeDirection)
            {
                robot.switch_dir();
            }

            // Move normally along walls
            robot.move();

            // Move down one strip and shift left/right after y_ref + 50
            if (robot.y == y_ref + 50)
            {
                // for y
                if (robot.clockwise)
                {
                    while (robot.detect_walls() != freeDirection)
                    {
                        robot.robotSensor(grid);
                        robot.x = robot.x - 1;
                        robot_pos.push_back({robot.x, robot.y});
                        limit_count++;
                    }
                    while (robot.detect_walls() == freeDirection)
                    {
                        robot.robotSensor(grid);
                        robot.x = robot.x - 1;
                        robot_pos.push_back({robot.x, robot.y});
                        limit_count++;
                    } // left wall

                    // for x
                }
                else if (!robot.clockwise)
                {
                    while (robot.detect_walls() != freeDirection)
                    {
                        robot.robotSensor(grid);
                        robot.x = robot.x + 1;
                        robot_pos.push_back({robot.x, robot.y});
                        limit_count++;
                    }
                    while (robot.detect_walls() == freeDirection)
                    {
                        robot.robotSensor(grid);
                        robot.x = robot.x + 1;
                        robot_pos.push_back({robot.x, robot.y});
                        limit_count++;
                    } // right wall
                }
                y_ref = robot.y;
                robot.switch_dir();
                std::cout << "Toggled direction! Now " << (robot.clockwise ? "CW" : "CCW") << std::endl;
            }
            if (robot.y >= max_y)
            {
                break;
            }
            prev_direction = robot.detect_walls();
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
