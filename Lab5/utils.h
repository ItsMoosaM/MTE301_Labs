// utility classes and functions
// All utility here only rely on already installed C++ libraries
#ifndef UTIL
#define UTIL

#include <random>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// functions
std::string trim(const std::string& );
// std::pair<std::string, bool> read_csv();
std::tuple<std::string, bool, int, int> read_csv();

class grid_util;    //forward declaration

// Object superclass
class Object {
    protected:
        int grid_value(grid_util&, void*, int, int, int);
    public:
        Object();
        Object(int, int, int, int);
        Object(int, int, int, int, int, int);
        int x, y;
        int width, height;
        int val;
        virtual int type ();
};

class Wall: public Object {
    public:
        Wall();
        Wall(float, float, float);
        void calc_dist(const Wall&);
        int type();
        float angle;
        int c_x, c_y;   //corners of the end for matching corners with next wall
};

std::vector<Object> create_walls (std::string);

class random_generator {
    std::random_device rd;                  // obtain a random number from hardware
    std::mt19937 gen;                       // seed the generator
    int env_size;   
    public:
        random_generator();
        int create_random(int, int);
};

class grid_util {
    //Occupancy grid; outer vector represents rows, inner represents columns along each row, initialized to 0's
    std::vector<std::vector<int>> grid;
    int env_width, env_height, min_obj_size, max_obj_size;
    // subgrids for making random spawns more efficient
    std::vector<std::vector<int>> subgrids;
    int num_subgrids, subgrids_size;
    friend class Object;
    private:
        int grid_value(int&&, int&&, int&, int&, int&);
        int max_y{0}, min_y{800};
        float wall_tol, wall_tol_b;
        std::vector<std::vector<int>> x_range;   //range of x values for a given y, starting at 0 for min_y
    public:
        // constructor
        grid_util(const int&, const int&, const int&, const int&, const int&, const int&);
        // functions
        // Object create_object(grid_util &, random_generator&, int, int, int, int, int, int);
        Object create_object(random_generator&, int, int, int, int, int, int);
        std::vector<Object *> create_objects (random_generator&, int, int);
        std::vector<Object *> create_objects_wall (random_generator&, int, int, int, int);
        void occupy_grid (int, int, int, int, int, int); 
        void occupy_grid_wall (Wall*, Wall*);
        bool is_occupied (int, int, int, int, int);
        int is_collision(Object);
        void writeGridToCSV(const std::string&);
        std::vector<Object *> create_walls (std::string);
        std::vector<Object *> create_angled_walls (std::string);
        // int grid_value(int, int, int);
        std::vector<std::vector<float>> read_csv(std::string, int);
        void clear_tol();
        void find_x_bounds();
        Object* spawn_object(random_generator&, const int&, int&&);
        float wall_accuracy(std::vector<std::vector<int>>);
        float grid_accuracy(std::vector<std::vector<int>>);
        // getters
        int get_min_y();
        int get_max_y();
};


#endif