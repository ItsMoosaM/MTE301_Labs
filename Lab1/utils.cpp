// utility classes and functions
// All utility here only rely on already installed C++ libraries

#include <random>
#include "utils.h"
#include <iostream>
#include <fstream>
#include <string>

random_generator::random_generator(): gen(rd()) {}

int random_generator::create_random(int lower_bnd, int upper_bnd) {
    std::uniform_int_distribution<> distr(lower_bnd, upper_bnd); // define the range
    return distr(gen);
}

grid_util::grid_util(int width, int height, int min_size, int max_size) : 
    env_width(width), 
    env_height(height),
    min_obj_size(min_size),
    max_obj_size(max_size)
{
    grid = std::vector<std::vector<int>>(env_height, std::vector<int>(env_width, 0));
}

Object grid_util::create_object(
    grid_util & grid, 
    random_generator &rand_gen, 
    int tol, int width, int height, int min, int max, int val, 
    std::string name) 
{
    Object rect;
    int x = rand_gen.create_random(0, env_width-width);
    int y = rand_gen.create_random(min, max);
    while (grid.is_occupied(tol, x, y, width, height)) {
        x = rand_gen.create_random(0, env_width);
        y = rand_gen.create_random(min, max);
    }
    grid.occupy_grid(tol, x, y, width, height, val, name);
    rect.x = x;
    rect.y = y;
    rect.width = width;
    rect.height = height;
    return rect;
}

std::vector<Object> grid_util::create_objects(random_generator &rand_gen, int tol, int num_objects) {
    std::vector<Object>objects(num_objects);
    // std::cout << "Creating " << num_objects << " rectangle objects in the environment" << std::endl;
    int obj_x, obj_y, obj_width, obj_height;
    for (int i = 0; i < num_objects; i++) {
        obj_x = rand_gen.create_random(0, env_width-max_obj_size); //x
        obj_y = rand_gen.create_random(0, env_height-max_obj_size); //y
        obj_width = rand_gen.create_random(min_obj_size, max_obj_size); //width
        obj_height = rand_gen.create_random(min_obj_size, max_obj_size); //height

        while (this->is_occupied(tol, obj_x, obj_y, obj_width, obj_height)) {
            obj_x = rand_gen.create_random(0, env_width-max_obj_size); //x
            obj_y = rand_gen.create_random(0, env_height-max_obj_size); //y
            obj_width = rand_gen.create_random(min_obj_size, max_obj_size); //width
            obj_height = rand_gen.create_random(min_obj_size, max_obj_size); //height
        }

        objects[i].x = obj_x; //x
        objects[i].y = obj_y; //y
        objects[i].width = obj_width; //width
        objects[i].height = obj_height; //height
        this->occupy_grid(tol, obj_x, obj_y, obj_width, obj_height, 2, "obstacle");
    }
    return objects;
}

// Occupy grid with values. -1 for tolerance bounds, 1 for robot, 2 for obstacles, 3 for goal
void grid_util::occupy_grid (int tol, int x, int y, int obj_width, int obj_height, int val, std::string name) 
{
    //Set min bounds in case x or y are less than occupancy tolerance (means -ve indices!)
    int min_bnd_x = (x < tol) ? 0 : x-tol;
    int min_bnd_y = (y < tol) ? 0 : y-tol;

    //Set max bounds in case x+tol or y+tol are out of bounds (seg fault!)
    int max_bnd_x = (env_width < x+obj_width+tol) ? env_width : x+obj_width+tol;
    int max_bnd_y = (env_height < y+obj_height+tol) ? env_height : y+obj_height+tol;

    for (int i=min_bnd_x; i<max_bnd_x; i++) {
        for (int j=min_bnd_y; j<max_bnd_y; j++) {
            if ((i<x) || (j<y)) {
                grid[i][j] = -1;
            }
            else if ((i>x+obj_width) || (j>y+obj_height)) {
                grid[i][j] = -1;
            }
            else {
                grid[i][j] = val;
            }
        }
    }
    // std::cout << "Created " << name << " at: (" << x << ", " << y << ") with width " << obj_width << " and height " << obj_height << std::endl;
}

bool grid_util::is_occupied (int tol, int x, int y, int width, int height) {
    int incr = min_obj_size+2*tol;
    // Go over grids efficiently by incrementing by min_obj_size + tolerance
    bool occupied;

    // For all x
    for (int i=x; i<x+width+1; i+=incr) {
        // For all y
        for (int j=y; j<y+height+1; j+=incr) {
            occupied = (grid[i][j] != 0) ? true : false;
            if (occupied == true) {
                return true;
            }
        }
        // Check if height%incr!=0 for the last y value of the object
        if (height%incr!=0) {
            occupied = (grid[i][y+height] != 0) ? true : false;
            if (occupied == true) {
                return true;
            }
        }
    }
    // Check if width%incr!=0 for the last x value of the object
    if (width%incr!=0) {
        for (int j=y; j<y+height+1; j+=incr) {
            occupied = (grid[x+width][j] != 0) ? true : false;
            if (occupied == true) {
                return true;
            }
        }
        if (height%incr!=0) {
            occupied = (grid[x+width][y+height] != 0) ? true : false;
            if (occupied == true) {
                return true;
            }
        }
    }
    return false;
}

// 0: no collision. 1: top left. 2: top right. 3: bottom left. 4: bottom right
// note due to the ordering of this, certain cases take precedence:
    // - for a hit to the full top side, top left will register first
    // - for a hit to the full left side, top left will register first
    // - for a hit to the full right side, top right will register first
    // - for a hit to the full bottom side, bottom left will register first
    // 9 possibilities
    // 0(no collision), 1(top), 2(left), 3(bottom), 4(right), 5(tl), 6(tr), 7(bl), 8(br)
    // 1+2=3, 1+3=4, 
int grid_util::is_collision (Object robot) {

    // Check the corners. If one of them is occupied by obstacle, it's collision
    // top left
    if (grid[robot.x][robot.y] == 2) {
        // top right
        if (grid[robot.x+robot.width][robot.y] == 2) {
            std::cout << "Collision at top, robot coordinates: " << robot.x << ", " << robot.y << std::endl;
            return 1;
        }
        // bottom left
        if (grid[robot.x][robot.y+robot.height] == 2) {
            std::cout << "Collision at left, robot coordinates: " << robot.x << ", " << robot.y << std::endl;
            return 2;
        }
        else {
            std::cout << "Collision at top left, robot coordinates: " << robot.x << ", " << robot.y << std::endl;
            return 5;
        }
    }
    // top right
    if (grid[robot.x+robot.width][robot.y] == 2) {
        // bottom right
        if (grid[robot.x+robot.width][robot.y+robot.height] == 2) {
            std::cout << "Collision at right, robot coordinates: " << robot.x << ", " << robot.y << std::endl;
            return 4;
        }
        else {
            std::cout << "Collision at top right, robot coordinates: " << robot.x << ", " << robot.y << std::endl;
            return 6;
        }

    }
    // bottom left
    if (grid[robot.x][robot.y+robot.height] == 2) {
        // bottom right
        if (grid[robot.x+robot.width][robot.y+robot.height] == 2) {
            std::cout << "Collision at bottom, robot coordinates: " << robot.x << ", " << robot.y << std::endl;
            return 3;
        }
        else {
            std::cout << "Collision at bottom, robot coordinates: " << robot.x << ", " << robot.y << std::endl;
            return 7;
        }
    }
    if (grid[robot.x+robot.width][robot.y+robot.height] == 2) {
        std::cout << "Collision at bottom right, robot coordinates: " << robot.x << ", " << robot.y << std::endl;
        return 8;
    }
    return 0;
}
// int grid_util::is_collision (Object robot) {

//     // Check the corners. If one of them is occupied by obstacle, it's collision
//     if (grid[robot.x][robot.y] == 2) {
//         std::cout << "Collision at top left: " << robot.x << ", " << robot.y << std::endl;
//         return 1;
//     }
//     if (grid[robot.x+robot.width][robot.y] == 2) {
//         std::cout << "Collision at top right: " << robot.x+robot.width << ", " << robot.y << std::endl;
//         return 2;
//     }
//     if (grid[robot.x][robot.y+robot.height] == 2) {
//         std::cout << "Collision at bottom left: " << robot.x << ", " << robot.y+robot.height << std::endl;
//         return 3;
//     }
//     if (grid[robot.x+robot.width][robot.y+robot.height] == 2) {
//         std::cout << "Collision at bottom right: " << robot.x+robot.width << ", " << robot.y+robot.height << std::endl;
//         return 4;
//     }
//     return 0;
// }

// Function to write a nested vector (grid) to a CSV file
void grid_util::writeGridToCSV(const std::string& filename) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    // Determine the maximum row size by finding the size of the longest inner vector
    size_t maxRowSize = 0;
    for (const auto& col : grid) {
        if (col.size() > maxRowSize) {
            maxRowSize = col.size();
        }
    }

    // Output the grid in transposed form (columns become rows in CSV)
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
    std::cout << "Grid written to " << filename << std::endl;
}
