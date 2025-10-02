// utility classes and functions
// All utility here only rely on already installed C++ libraries
#ifndef UTIL
#define UTIL

#include <random>
#include <iostream>

struct Object {
    int x, y, width, height;
};

class random_generator {
    std::random_device rd;                  // obtain a random number from hardware
    std::mt19937 gen;                       // seed the generator
    int env_size;   
    public:
        random_generator();
        int create_random(int, int);
};

class grid_util {
    int env_width, env_height, min_obj_size, max_obj_size;
    //Occupancy grid; outer vector represents rows, inner represents columns along each row, initialized to 0's
    
    public:
        std::vector<std::vector<int>> grid;
        grid_util(int, int, int, int);
        Object create_object(grid_util &, random_generator&, int, int, int, int, int, int, std::string);
        std::vector<Object> create_objects (random_generator&, int, int);
        void occupy_grid (int, int, int, int, int, int, std::string); 
        bool is_occupied (int, int, int, int, int);
        int is_collision(Object);
        void writeGridToCSV(const std::string&);
};


#endif