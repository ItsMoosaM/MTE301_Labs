// utility classes and functions
// All utility here only rely on already installed C++ libraries

#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

#include "utils.h"

random_generator::random_generator(): gen(rd()) {}

int random_generator::create_random(int lower_bnd, int upper_bnd) {
    std::uniform_int_distribution<> distr(lower_bnd, upper_bnd); // define the range
    return distr(gen);
}

// constructor
grid_util::grid_util(int width, int height, int min_size, int max_size) : 
    env_width(width), 
    env_height(height),
    min_obj_size(min_size),
    max_obj_size(max_size)
{
    grid = std::vector<std::vector<int>>(env_height, std::vector<int>(env_width, 0));
    // grid is square, so subgrids are also square
    this->num_subgrids = 4;
    int num_subgrids_x = std::sqrt(this->num_subgrids);
    this->subgrids_size = env_width/num_subgrids_x;

    int x = 0, y = 0;
    // subgrids has 3 elements (O, x, y) where O counts how many objects are in grid: 0, 1, 2, etc..
    // populate subgrids from left-right(x+), then top-down(y+), starting from top row
    for (int j = 0; j<num_subgrids_x; j++) {
        for (int i = 0; i<num_subgrids_x; i++) {
            x = i*this->subgrids_size;
            y = j*this->subgrids_size;
            subgrids.push_back({0, x, y});
        }
    }
}

// create a single object. for robot and goal.
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

// create multiple objects. for the obstacles
std::vector<Object> grid_util::create_objects(random_generator &rand_gen, int tol, int num_objects) {
    std::vector<Object>objects(num_objects);
    int obj_x, obj_y, obj_width, obj_height;
    bool limit_reached=false;
    int max_iter = 0;
    int min_x, min_y, max_x, max_y;
    for (int i = 0; i < num_objects; i++) {
        // set min and max for subgrids to spawn in. Based on subgrid 0, the one with lowest occupancy
        min_x = this->subgrids[0][1] < tol ? tol: this->subgrids[0][1];
        min_y = this->subgrids[0][2] < tol ? tol: this->subgrids[0][2];
        // ensure top left is within bounds+tolerance if it's the rightmost/downmost subgrids
        max_x = this->subgrids[0][1] + this->subgrids_size == env_width 
            ? this->subgrids[0][1] + this->subgrids_size - max_obj_size - tol 
            : this->subgrids[0][1] + this->subgrids_size;
        max_y = this->subgrids[0][2] + this->subgrids_size == env_height 
            ? this->subgrids[0][2] + this->subgrids_size - max_obj_size - tol 
            : this->subgrids[0][2] + this->subgrids_size;

        // initial positions/sizes
        obj_x = rand_gen.create_random(min_x, max_x); //x
        obj_y = rand_gen.create_random(min_y, max_y); //y
        obj_width = rand_gen.create_random(min_obj_size, max_obj_size); //width
        obj_height = rand_gen.create_random(min_obj_size, max_obj_size); //height
        
        // re-initialize position/sizes until it spawns. try 10,000 times for each object
        while (this->is_occupied(tol, obj_x, obj_y, obj_width, obj_height)) {
            obj_x = rand_gen.create_random(min_x, max_x); //x
            obj_y = rand_gen.create_random(min_y, max_y); //y
            obj_width = rand_gen.create_random(min_obj_size, max_obj_size); //width
            obj_height = rand_gen.create_random(min_obj_size, max_obj_size); //height
            max_iter++;
            if (max_iter>=10000) {
                limit_reached = true;
                break;
            }
        }
        max_iter = 0;
        if (limit_reached) {
            // std::cout << "no space to spawn object number " << i+1 << " after 10000 tries." << std::endl;
            limit_reached = false;
            continue;
        }
        objects[i].x = obj_x; //x
        objects[i].y = obj_y; //y
        objects[i].width = obj_width; //width
        objects[i].height = obj_height; //height
        this->occupy_grid(tol, obj_x, obj_y, obj_width, obj_height, 2, "obstacle");

        // sort the subgrid based on lowest occupancy
        std::stable_sort(subgrids.begin(), subgrids.end(), [](const std::vector<int>& a, const std::vector<int>& b) {
            return a[0] < b[0];});
    }
    // for (auto &it: this->subgrids) {
    //     std::cout << "Subgrid at " << it[1] << ", " << it[2] << " has " << it[0] << " objects." << std::endl;
    // }    
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
    // check which subgrids the object occupies. An object can occupy multiple subgrids
    bool x_intersect, y_intersect;
    for (auto &it: this->subgrids) {
        if (x <= it[1]+this->subgrids_size && it[1] <= x+obj_width) {
            x_intersect = true;
        }
        if (y <= it[2]+this->subgrids_size && it[2] <= y+obj_height) {
            y_intersect = true;
        }
        if (x_intersect && y_intersect) {
            it[0] += 1;
            // std::cout << "Subgrid at " << it[1] << ", " << it[2] << " occupied." << std::endl;
        }
        // reset
        x_intersect = false;
        y_intersect = false;
    }
}

// check if portion of grid is occupied before spawning an object
bool grid_util::is_occupied (int tol, int x, int y, int width, int height) {

    // increment size is min object size + 2*tol (on each side)
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

// Function to write a nested vector (grid) to a CSV file. In case students want to analyze the grid as csv
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
