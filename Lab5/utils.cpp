// utility classes and functions
// All utility here only rely on already installed C++ libraries

#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <memory>
#include <cassert>

#include "utils.h"

random_generator::random_generator(): gen(rd()) {}

int random_generator::create_random(int lower_bnd, int upper_bnd) {
    std::uniform_int_distribution<> distr(lower_bnd, upper_bnd); // define the range
    return distr(gen);
}

// helper function to trim whitespace
std::string trim(const std::string& str) {
    const auto strBegin = str.find_first_not_of(" \t");
    const auto strEnd = str.find_last_not_of(" \t");
    const auto strRange = strEnd - strBegin + 1;

    return strBegin == std::string::npos ? "" : str.substr(strBegin, strRange);
}

std::tuple<std::string, bool, int, int> read_csv() {
    // tuple to store the return values: (csv_file, render_ground_truth, render_speed)
    std::tuple<std::string, bool, int, int> config_values;
    
    // variables to store parsed values
    std::string csv_file;
    bool render_ground_truth = false;
    int render_speed = 0;  // default value for render_speed

    // open the config file
    std::ifstream config("config.csv");
    if (!config.is_open()) {
        std::cerr << "Error: Could not open config.csv file." << std::endl;
        return config_values;
    }

    std::string line;
    while (std::getline(config, line)) {
        // use a stringstream to split the line by commas
        std::istringstream iss(line);
        std::string key, value;

        // parse key and value separated by a comma
        if (std::getline(iss, key, ',') && std::getline(iss, value)) {
            // trim whitespace from both key and value
            key = trim(key);
            value = trim(value);

            // handle the key-value pairs
            if (key == "file") {
                csv_file = value;
                std::get<0>(config_values) = csv_file;
            } else if (key == "render_ground_truth") {
                render_ground_truth = (value == "1");  // convert "1" to true, "0" to false
                std::get<1>(config_values) = render_ground_truth;
            } else if (key == "render_speed") {
                try {
                    render_speed = std::stoi(value);  // convert the string to an integer
                    std::get<2>(config_values) = render_speed;
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Error: Invalid render_speed value." << std::endl;
                }
            }
        }
    }

    config.close();
    // read environment csv file and determine no. of columns
    std::ifstream file(std::get<0>(config_values));

    // if file can't open, return empty nested vector
    if (!file.is_open()) {
        std::cerr << "Could not open the file: " << std::get<0>(config_values) << std::endl;
    }
    // read first line from the file to determine no. of columns
    std::getline(file, line); 
    std::stringstream ss(line);
    std::string value;
    int col{0};

    // Read 3/4 comma-separated values from the line
    while (std::getline(ss, value, ',')) {
        col++;
    }
    std::get<3>(config_values) = col;
    return config_values;
}


// robot superclass
// default constructor
Object::Object(){}
// constructor with four arguments
Object::Object(int width, int height, int env_width, int env_height):
    width(width),
    height(height)
{
    random_generator rand_gen;
    // spawn within a 200 pixel band from the center
    x = rand_gen.create_random(env_width/2 - 10, env_width/2 + 10);
    y = rand_gen.create_random(env_height/2 - 200, env_height/2 - 50);
    // std::cout << "Robot spawned with x, y: " << x << ", " << y << std::endl;
}
// constructor with six arguments for lab 4
Object::Object(int width, int height, int env_width, int min_y, int max_y, int tol):
    width(width),
    height(height)
{
    random_generator rand_gen;
    x = rand_gen.create_random(env_width/2 - 50, env_width/2 + 50);
    // choose y to be within a 20 pixel
    y = rand_gen.create_random(max_y-(2*width)-tol, max_y-width-tol);
}

int Object::grid_value(grid_util& grid, void* obj, int x, int y, int range) {
    Object* ptr = static_cast<Object*>(obj); // cast from my_robot subclass to Object
    if (ptr) {
        // pass the center point as the x,y coordinate of the robot
        return grid.grid_value(ptr->x+ptr->width/2, ptr->y+ptr->height/2, x, y, range);
        // std::cout << "Successfully cast from my_robot to Object!" << std::endl;
    }
    else {
        std::cout << "Cast failed!" << std::endl;
        return -1;
    }
    return -1;
}

// virtual function for the sake of dynamic casting to Wall in the render function
int Object::type() {
    return 0;
}

// wall subclass of Object
// default constructor
Wall::Wall(){}
Wall::Wall(float width, float height, float angle):
    Object()
{
    this->width = round(width);
    this->height = round(height);
    this->angle = angle;
    // std::cout << "wall created with width, height, angle: " << this->width << ", " << this->height << ", " << this->angle << std::endl;
}

void Wall::calc_dist(const Wall &wall) {
    this->x = round(wall.x +std::cos(wall.angle*(M_PI / 180.f))*wall.width/2 
        + std::cos(this->angle*(M_PI / 180.f))*this->width/2);
    this->y = round(wall.y +std::sin(wall.angle*(M_PI / 180.f))*wall.width/2 
        + std::sin(this->angle*(M_PI / 180.f))*this->width/2);
    // if angle >= 135, subtract cos45*thickness of wall
    if (abs(this->angle - wall.angle)>=135) {
        // if it's 45 -> 180, -x, +y
        // if it's 180 -> 315, -x, -y
        // if first angle <= 45, go down y
        if (wall.angle <= 45) {
            // this->y += std::cos(45.0*(M_PI / 180.f))*wall.height;
            this->y += wall.height;
        }
        // if it's 90-135, move left
        // if it's 180-215, go up y
        if (180 <= wall.angle && wall.angle <= 215) {
            // this->y -= std::cos(45.0*(M_PI / 180.f))*wall.height;
            this->y -= wall.height;
        }
        
    }
    
}

int Wall::type() {
    return 1;
}

// constructor
grid_util::grid_util(
    const int& width, const int& height, const int& min_size,  const int& max_size, 
    const int& radius, const int& r_tol): 
    env_width(width), 
    env_height(height),
    min_obj_size(min_size),
    max_obj_size(max_size)
{
    grid = std::vector<std::vector<int>>(env_height, std::vector<int>(env_width, 0));
    
    float c_45 = std::cos(45*M_PI/180.f);
    //tolerance between object and wall, considering robots following angled walls
    this->wall_tol = ceil(static_cast<float>(2*(radius+r_tol))/c_45);  
    //tolerance between object and wall only at the robot, considering robot spawn point      
    this->wall_tol_b = 4*radius+(2*r_tol);

    //============SUBGRIDS=============
    // grid is square, so subgrids are also square
    this->num_subgrids = 4;
    int num_subgrids_x = std::sqrt(this->num_subgrids);
    // make subgrid only 400x400
    this->subgrids_size = env_width/(2*num_subgrids_x);
    int x = 0, y = 0;
    int start_x{200}, start_y{200};
    // subgrids has 3 elements (O, x, y) where O counts how many objects are in grid: 0, 1, 2, etc..
    // populate subgrids from left-right(x+), then top-down(y+), starting from top row
    for (int j = 0; j<num_subgrids_x; j++) {
        for (int i = 0; i<num_subgrids_x; i++) {
            x = i*this->subgrids_size + start_x;
            y = j*this->subgrids_size + start_y;
            subgrids.push_back({0, x, y});
        }
    }


}

// create a single object. for robot and goal.
Object grid_util::create_object(
    // grid_util & grid, 
    random_generator &rand_gen, 
    int tol, int width, int height, int min, int max, int val) 
{
    Object rect;
    int x = rand_gen.create_random(0, env_width-width);
    int y = rand_gen.create_random(min, max);
    // while (grid.is_occupied(tol, x, y, width, height)) {
    while (this->is_occupied(tol, x, y, width, height)) {
        x = rand_gen.create_random(0, env_width);
        y = rand_gen.create_random(min, max);
    }
    // grid.occupy_grid(tol, x, y, width, height, val);
    this->occupy_grid(tol, x, y, width, height, val);
    rect.x = x;
    rect.y = y;
    rect.width = width;
    rect.height = height;
    return rect;
}

// create multiple objects. for the obstacles
std::vector<Object *> grid_util::create_objects(random_generator &rand_gen, int tol, int num_objects) {
    std::vector<Object *>objects;
    int obj_x, obj_y, obj_width, obj_height;
    bool limit_reached=false;
    int max_iter = 0;
    int min_x, min_y, max_x, max_y;
    for (int i = 0; i < num_objects; i++) {
        Object *obj = new Object;   //allocate new
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
            std::cout << "no space to spawn object number " << i+1 << " after 10000 tries." << std::endl;
            limit_reached = false;
            continue;
        }
        obj->x = obj_x;
        obj->y = obj_y;
        obj->width = obj_width;
        obj->height = obj_height;
        objects.push_back(obj);
        this->occupy_grid(tol, obj_x, obj_y, obj_width, obj_height, 2);
        // std::cout << "Object spawned at: " << obj->x << ", " << obj->y << std::endl;

        // sort the subgrid based on lowest occupancy
        std::stable_sort(subgrids.begin(), subgrids.end(), [](const std::vector<int>& a, const std::vector<int>& b) {
            return a[0] < b[0];});
    }
    // for (auto &it: this->subgrids) {
    //     std::cout << "Subgrid at " << it[1] << ", " << it[2] << " has " << it[0] << " objects." << std::endl;
    // }    
    return objects;
}

std::vector<Object *> grid_util::create_objects_wall(
    random_generator &rand_gen, 
    int o_tol, int r_tol, int radius, 
    int num_objects) {
    bool obj_found_l{false};
    bool obj_found_r{false};
    bool limit_reached=false;
    int max_iter = 0;

    std::vector<Object *> objects;

    int obj_x, obj_y, obj_width, obj_height;

    for (int i = 0; i < num_objects; i++) {
        Object *obj = this->spawn_object(rand_gen, o_tol, 1);
        if (obj) {
            objects.push_back(obj);
        }
        // else {
        //     // std::cout << "No valid object returned!\n";
        // }
    }
    return objects;
}

//spawn a single object: obstacle or goal, inside the walls
Object* grid_util::spawn_object(random_generator &rand_gen, const int& o_tol, int&& val) {
    bool obj_found_l{false};
    bool obj_found_r{false};
    int limit_reached{false};
    Object *obj = new Object;   //allocate new
    int max_iter=0;
    int obj_x, obj_y, obj_width, obj_height;

    int max_y_obj;
    int min_y_obj = this->min_y + this->wall_tol;

    // if obstacle, min/max y are bounded by upper/lower walls
    if (val == 1) {
        max_y_obj = this->max_y - this->wall_tol_b - max_obj_size;
    }
    // if goal, it's bottom is higher
    else {
        max_y_obj = min_y_obj + this->max_obj_size;

    }

    do {
        // initial positions/sizes
        obj_y = rand_gen.create_random(min_y_obj, max_y_obj); //y
        obj_width = rand_gen.create_random(min_obj_size, max_obj_size); //width
        obj_height = rand_gen.create_random(min_obj_size, max_obj_size); //height
        
        // given y, now create x range, counting from inside to out
        int x_left{0}, x_right{0};
        int i_l{399};   //counter from middle to left
        int left_wall{0};   //min x (i.e. left side of wall)
        for (int i_r=400; i_r<this->grid.size(); i_r++) {   //i_l is counter from middle to right
            Object *obj = new Object;   //allocate new
            if (this->grid[i_r][obj_y]!=0) {
                obj_found_r = true;
            }
            else if (this->grid[i_r][obj_y]==0 && !obj_found_r) {
                x_right++;
            }
            if (this->grid[i_l][obj_y]!=0) {
                obj_found_l = true;
            }
            else if (this->grid[i_r][obj_y]==0 && !obj_found_l) {
                x_left++;
            }
            i_l--;
        }
        //reset to false
        obj_found_l = false;
        obj_found_r = false;
        // define the x range

        int min_x_obj = 399-x_left;
        int max_x_obj = 400+x_right;
        // add wall tolerance to left side if min x is left wall
        if (this->x_range[obj_y - this->min_y][0]>=min_x_obj) {
            min_x_obj += this->wall_tol;
        }
        // subtract wall tolerance to right side if max x is right wall
        if (this->x_range[obj_y - this->min_y][1]<=max_x_obj+obj_width) {
            max_x_obj -= (this->wall_tol+obj_width);
        }
        assert(max_x_obj>min_x_obj);
        obj_x = rand_gen.create_random(min_x_obj, max_x_obj);
        max_iter++;
        if (max_iter>=10000) {
            limit_reached = true;
            break;
        }

    } while (this->is_occupied(o_tol, obj_x, obj_y, obj_width, obj_height));

    if (limit_reached) {
        // std::cout << "no space to spawn object after 10000 tries." << std::endl;
        limit_reached = false;
        return nullptr;
    }
    obj->x = obj_x;
    obj->y = obj_y;
    obj->width = obj_width;
    obj->height = obj_height;
    obj->val = val;
    this->occupy_grid(o_tol, obj_x, obj_y, obj_width, obj_height, val);
    return obj;
}

//for a given y, find the x bounds
void grid_util::find_x_bounds() {
    bool wall_found_l{false};
    bool wall_found_r{false};

    for (int j=this->min_y; j<this->max_y; j++) {
        int i_l{399};
        int i_r;
        for (int i=400; i<this->grid.size(); i++) {
            if (this->grid[i][j]) {
                wall_found_r = true;
            }
            if (this->grid[i_l][j]) {
                wall_found_l = true;
            }
            if (wall_found_l && wall_found_r) {
                i_r = i;
                break;
            }
            if (wall_found_r) {
                i_r = i;
            }
            if (!wall_found_l) {
                i_l--;
            }
        }

        this->x_range.push_back({i_l, i_r});
        // reset
        wall_found_l = false;
        wall_found_r = false;
    }
}

// occupy grid with angled walls
void grid_util::occupy_grid_wall (Wall *wall_prev, Wall *wall)
{
    // angle in radians
    float angRad = wall->angle*(M_PI / 180.f);
    // angle alpha (90 - angle) in radians
    float alphaRad = (90 - wall->angle)*(M_PI / 180.f);

    // calculate 6 corner points with 6 x's and 4 y's:
        // (x1, y1), (x2_l, y2), (x2_r, y2), (x3_l, y3), (x3_r, y3), (x4, y4)
    float x1, y1, x2_l, x2_r, y2, x3_l, x3_r, y3, x4, y4;

    //main steps for points of angled wall (e.g. for 45 degrees): 
        // 1) pivot corner(top) matches corner of previous entry and adjust
        // 2) from pivot corner, next two corners are (+14, +14) and (-13, +14) in x/y
            //note: pivot corner doesn't actually get filled
        // 3) readjust center origin to be (+70, +70) from top corner
        // 4) make the next two bottom corners to be (+43, +70) (+70, +70) from new origin
            // note these corners are (+56, +56) from the respective corners in 2
        // 5) make bottom corner to be (+14, +14) from left corner of last step
        // 6) now fill grids, all sides inclusive

    // define the predefined lengths of slanted wall. might have to change for 135 due to angle
    int diag_width = floor(abs(2*wall->height*std::cos(angRad))); //28
    int diag_length = floor(abs(wall->width*std::cos(angRad)));   //141
    
    if (wall->angle == 45.0) {
        //1) simply make the first corner equal to previous corner
        x1 = wall_prev->c_x;
        y1 = wall_prev->c_y;
        //2) next two corners
        // might have to change for 135
        x2_l = x1 - diag_width/2 + 1;
        x2_r = x1 + diag_width/2;
        y2 = y1 + diag_width/2;
        //3) readjust center
        wall->x = x1 + diag_length/2;
        wall->y = y1 + diag_length/2;
        //4) next two corners
        x3_r = wall->x + diag_length/2;
        x3_l = x3_r - diag_width+1;
        y3 = wall->y + diag_length/2;
        //5) bottom corner
        x4 = x3_l + diag_width/2;
        y4 = y3 + diag_width/2;
        // adjust end corner to be reference previous corner for the next wall
        wall->c_x = x3_r;
        wall->c_y = y3;
    }
    else if (wall->angle == 135.0) {
        //1) make 2nd top, right corner equal to previous corner
        x2_r = wall_prev->c_x;
        y2 = wall_prev->c_y;
        //2) top corner, and 2nd top left corner after that
        x1 = x2_r - diag_width/2 + 1;
        y1 = y2 - diag_width/2;
        x2_l = x1 - diag_width/2;
        //3) readjust center
        wall->x = x2_r - diag_length/2;
        wall->y = y2 + diag_length/2;
        //4) bottom corner
        x4 = wall->x - diag_length/2;
        y4 = wall->y + diag_length/2;
        //5) next two corners
        x3_r = x4 + diag_width/2;
        x3_l = x3_r - diag_width + 1;
        y3 = y4 - diag_width/2;
        // adjust end corner to be reference previous corner for the next wall
        wall->c_x = x4;
        wall->c_y = y4;
    }
    else if (wall->angle == 225.0) {
        //1) make bottom corner equal to previous corner
        x4 = wall_prev->c_x;
        y4 = wall_prev->c_y;
        //2) second bottom row corners after that
        x3_l = x4 - diag_width/2;
        x3_r = x3_l + diag_width - 1;
        y3 = y4 - diag_width/2;
        //3) readjust center
        wall->x = x4 - diag_length/2;
        wall->y = y4 - diag_length/2;
        //4) second top row corners
        x2_l = wall->x - diag_length/2;
        y2 = wall->y - diag_length/2;
        x2_r = x2_l + diag_width - 1;
        //5) top corner
        x1 = x2_l + diag_width/2;
        y1 = y2 - diag_width/2;
        // adjust end corner to be reference previous corner for the next wall
        wall->c_x = x2_l;
        wall->c_y = y2;
    }
    else if (wall->angle == 315.0) {
        //1) make 2nd bottom, left corner equal to previous corner
        x3_l = wall_prev->c_x;
        y3 = wall_prev->c_y;
        x3_r = x3_l + diag_width - 1;
        // if previous wall was 180
        if (wall_prev->angle == 180) {
            y3 -= wall_prev->height;
        }
        //2) bottom corner
        x4 = x3_l + diag_width/2 - 1;
        y4 = y3 + diag_width;
        //3) readjust center
        wall->x = x3_l + diag_length/2;
        wall->y = y3 - diag_length/2;
        //4) top corner
        x1 = wall->x + diag_length/2;
        y1 = wall->y - diag_length/2;
        //5) 2nd top row corners
        x2_l = x1 - diag_width/2;
        x2_r = x2_l + diag_width - 1;
        y2 = y1 + diag_width/2;
        // adjust top corner to be reference previous corner for the next wall
        wall->c_x = x1;
        wall->c_y = y1;
    }

    // top triangle
    int x_min = x2_l, x_max = x2_r;
    for (int j = y2; j>y1; j--) {
        for (int i = x_min; i<=x_max; i++) {
            this->grid[i][j] = 1;
        }
        // reduce by 1 pixel both sides
        x_min += 1;
        x_max -= 1;
    }
    
    // bottom triangle
    x_min = x3_l, x_max = x3_r;
    for (int j = y3; j<y4; j++) {
        for (int i = x_min; i<=x_max; i++) {
            this->grid[i][j] = 1;
        }
        // reduce by 1 pixel both sides
        x_min += 1;
        x_max -= 1;
    }

    // middle section
    x_min = x2_l, x_max = x2_r;
    for (int j = y2; j <= y3; j++) {
        for (int i = x_min; i <= x_max; i++) {
            this->grid[i][j] = 1;
        }
        // shift one to the right
        if (wall->angle==45.0 || wall->angle==225.0) {
            x_min+=1, x_max+=1;
        }
        // shift one to the left
        else if (wall->angle==135.0 || wall->angle==315.0) {
            x_min-=1, x_max-=1;
        }
    }
} 

// Occupy grid with values. -1 for tolerance bounds, 1 for robot, 2 for obstacles, 3 for goal
void grid_util::occupy_grid (int tol, int x, int y, int obj_width, int obj_height, int val) 
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
    // 0(no collision            // if (this->grid[i][j] == -1) {
            //     this->grid[i][j] = 0;
            // }), 1(top), 2(left), 3(bottom), 4(right), 5(tl), 6(tr), 7(bl), 8(br)
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

// function to write a nested vector (grid) to a CSV file. In case students want to analyze the grid as csv
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

// read csv and return entries
std::vector<std::vector<float>> grid_util::read_csv(std::string file_name, int length) {
    // nested vector to store the size 4 vectors
    std::vector<std::vector<float>> data;
    // open the file
    std::ifstream file(file_name);

    // if file can't open, return empty nested vector
    if (!file.is_open()) {
        std::cerr << "Could not open the file: " << file_name << std::endl;
        return data;
    }

    std::string line;
    
    // Read each line from the file
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<float> row;
        std::string value;
        float num;

        // Read 3/4 comma-separated values from the line
        while (std::getline(ss, value, ',')) {
            num = std::stof(value);  // Convert string to float
            row.push_back(num);
        }

        // Ensure the row has exactly 4 values before pushing it to the nested vector
        if (row.size() == length) {
            data.push_back(row);
        } else {
            std::cerr << "Invalid row size: " << row.size() << ". Skipping line." << std::endl;
        }
    }

    // Close the file
    file.close();
    return data;
}

// simple function for creating walls from csv
std::vector<Object*> grid_util::create_walls (std::string file_name) {
    // create the walls
    std::vector<Object*> walls;

    std::vector<std::vector<float>> data = this->read_csv(file_name, 4);
    int thick;
    // Print out the nested vector to check the data
    for (const auto& vec : data) {
        // std::cout << "x: " << vec[0] << ", y: " << vec[1] << ", width: " << vec[2] << ", height: " << vec[3] << std::endl;
        Object* wall = new Object;
        wall->x = vec[0];
        wall->y = vec[1];
        wall->width = vec[2];
        wall->height = vec[3];
        wall->val = 1;
        walls.push_back(wall);
        this->occupy_grid(0, wall->x, wall->y, wall->width, wall->height, 1);
        if (wall->y>this->max_y) {
            this->max_y = wall->y;
        }
        if (wall->y<this->min_y) {
            this->min_y = wall->y;
        }
        thick = wall->height;
    }
    // subtract wall thickness to get min/max y
    this->min_y += thick;
    this->max_y -= thick;
    // for every y inside wall, find x_min and x_max
    this->find_x_bounds();
    return walls;
}

// create angled walls, fill the grid with them adjusted for symmetry, and return a vector of unadjusted walls for SFML
std::vector<Object*> grid_util::create_angled_walls (std::string file_name) {

    std::vector<Object*> walls_render;  // vector of Object* to store walls to render.

    std::vector<std::vector<float>> data = this->read_csv(file_name, 3);

    Wall default_wall, default_wall_r;
    Wall* wall_prev = &default_wall;  // use Wall* to store the previous wall
    Wall* wall_prev_r = &default_wall_r;  // use Wall* to store the previous render wall
    int dx, dy; // (x, y) corner of previous wall. corresponds to (x+width/2, y) from origin if it was 0 degree wall
    int thick;
    for (auto vec = data.begin(); vec != data.end(); vec++) {
        Wall* wall = new Wall((*vec)[0], (*vec)[1], (*vec)[2]);  // dynamically allocate Wall
        Wall *wall_r;
        if (vec != data.begin()) {
            wall_r = new Wall(*wall); // copy wall for rendering before adjusting
            wall->calc_dist(*wall_prev);  // calculate distance of center based on previous wall
            wall_r->calc_dist(*wall_prev_r);  // calculate distance of center based on previous wall
            if (wall_r->angle==135.0 && file_name == "environment2.csv") {
                wall_r->y+=1;
            }
            // auto index = std::distance(data.begin(), vec); // for printing next line
            // std::cout << "Wall no." << index + 1 << " created with (x, y): " << wall->x << ", " << wall->y << std::endl;
        } else {
            wall->x = this->env_width/2;
            wall->y = this->env_height/2 - 241;
            // initialize previous values, assuming first wall is angle 0 degrees
            wall_prev->c_x = wall->x - wall->width/2;
            wall_prev->c_y = wall->y;
            // copy wall for rendering
            wall_r = new Wall(*wall); 
            // std::cout << "First wall created with (x, y): " << wall->x << ", " << wall->y << std::endl;
        }
        if (wall->angle == 0) {
            // compare first corner with last corner of previous wall
            dx = wall_prev->c_x - (wall->x-wall->width/2); //100 to the left of center
            dy = wall_prev->c_y - wall->y;
            // adjust the center
            wall->x += dx;
            wall->y += dy;
            // std::cout << "Adjusting center of angle 0 wall by: " << dx << ", " << dy << std::endl;
            this->occupy_grid(0, wall->x-wall->width/2, wall->y, wall->width, wall->height, 1);
            wall->c_x = wall->x+wall->width/2 - 1; //+99 instead of 100 (middle is on right side)
            wall->c_y = wall->y;
        }
        else if (wall->angle == 90) {
            // compare first corner with last corner of previous wall. 
            dx = wall_prev->c_x - wall->x; 
            dy = wall_prev->c_y - (wall->y - wall->width/2); //100 to the top of center
            // adjust the center
            wall->x += dx;
            wall->y += dy;
            // std::cout << "Adjusting center of angle 90 wall by: " << dx << ", " << dy << std::endl;
            this->occupy_grid(0, wall->x-wall->height+1, wall->y-wall->width/2, wall->height, wall->width, 1);
            wall->c_x = wall->x;
            wall->c_y = wall->y + wall->width/2 - 1;
        }
        else if (wall->angle == 180) {
            // compare first corner with last corner of previous wall. 
            dx = wall_prev->c_x - (wall->x + wall->width/2 - 1); //99 to the right of center
            dy = wall_prev->c_y - wall->y;
            // adjust the center
            wall->x += dx;
            if (wall_prev->angle > 45) {
                wall->y += dy;
            }
            // std::cout << "Adjusting center of angle 180 wall by: " << dx << ", " << dy << std::endl;
            this->occupy_grid(0, wall->x-wall->width/2, wall->y-wall->height+1, wall->width, wall->height, 1);
            wall->c_x = wall->x - wall->width/2;
            wall->c_y = wall->y;
            // slight adjustment for gt render
            if (file_name == "environment1.csv") {
                wall_r->width += 0;
                wall_r->x -= 0;
            }
            
        }
        else if (wall->angle == 270) {
            // compare first corner with last corner of previous wall. 
            dx = wall_prev->c_x - wall->x; 
            dy = wall_prev->c_y - (wall->y + wall->width/2 - 1); //99 to the bottom of center
            // adjust the center
            wall->x += dx;
            wall->y += dy;
            // std::cout << "Adjusting center of angle 270 wall by: " << dx << ", " << dy << std::endl;
            this->occupy_grid(0, wall->x, wall->y-wall->width/2, wall->height, wall->width, 1);
            wall->c_x = wall->x;
            wall->c_y = wall->y - wall->width/2; //100 to the top of center
        }
        else { 
            // std::cout << "angled wall origin: " << wall->x << ", " << wall->y << std::endl;
            this->occupy_grid_wall(wall_prev, wall);
        }

        if (wall->y>this->max_y) {
            this->max_y = wall->y;
        }
        if (wall->y<this->min_y) {
            this->min_y = wall->y;
        }
        walls_render.push_back(wall_r);
        wall_prev = wall;        // update wall_prev
        wall_prev_r = wall_r;
        thick = wall->height;
        delete wall;
    }
    // subtract wall thickness to get min/max y
    this->min_y += thick;
    this->max_y -= thick;
    // for every y inside wall, find x_min and x_max
    this->find_x_bounds();
    return walls_render;
}

// clear the -1 tolerance values after all objects have been spawned
void grid_util::clear_tol() {
    for (int i = 0; i < this->grid.size(); i++) {
        for (int j = 0; j < this->grid[0].size(); j++) {
            // if grid value is -1, make it 0, else stays the same
            this->grid[i][j] = (this->grid[i][j]==-1)? 0: this->grid[i][j];
        }
    }
}

int grid_util::get_min_y() {
    return this->min_y;
}

int grid_util::get_max_y() {
    return this->max_y;
}

// original version of grid_value that is public
// int grid_util::grid_value(int x, int y, int range) {
//     if ((0 <= x && x < this->env_width) && (0 <= y && y < this->env_height)) {
//         // make the max range the smaller of the range provided, or 50
//         int max_range = std::min(range, 50);
//         return this->grid[x][y];
        
//     }
//     else {
//         std::cerr << "Accessing element out of the grid boundaries" << std::endl;
//         return -1;
//     }
// }

// private version only accessed by friend class Object
int grid_util::grid_value(int &&x, int &&y, int &x_q, int &y_q, int &range) {
    // check if query point is within bounds
    if ((0 <= x_q && x_q < this->env_width) && (0 <= y_q && y_q < this->env_height)) {
        // make the max range the smaller of the range provided, or 50
        int max_range = std::min(range, 50);
        // if ((abs(x_q-x)>max_range) || (abs(y_q-y)>max_range)) {
        if ((x_q-x)*(x_q-x) + (y_q-y)*(y_q-y) > (max_range*max_range)) {
            std::cerr << "==(grid_util)==: Trying to access grid value beyond robot sensor range of 50!==" << std::endl;
            std::cout << "Current robot position (x, y): " << x << ", " << y << std::endl;
            std::cout << "Current query location (x, y): " << x_q << ", " << y_q << std::endl;
            return -1;
        }
        else {
            return this->grid[x_q][y_q];
        }
    }
    else {
        std::cerr << "Accessing element out of the grid boundaries" << std::endl;
        return -1;
    }
}

float grid_util::grid_accuracy(std::vector<std::vector<int>> grid_pred) {
    float correct_count = 0.0;
    float count = 0.0;
    bool wall_found_l{false};
    bool wall_found_r{false};

    for (int j=0; j<this->grid.size(); j++) {
        int i_b{799};
        for (int i=0; i<this->grid.size()/2; i++) {
            if (this->grid[i][j]) {
                wall_found_l = true;
                count ++;
            }
            if (this->grid[i_b][j]) {
                wall_found_r = true;
                count ++;
            }
            if (wall_found_l) {
                correct_count += (this->grid[i][j] == grid_pred[i][j]);
                // for the insides of the walls which are = 0
                if (!this->grid[i][j]) {
                    count++;
                }
            }
            if (wall_found_r) {
                correct_count += (this->grid[i_b][j] == grid_pred[i_b][j]);
                // for the insides of the walls which are = 0
                if (!this->grid[i_b][j]) {
                    count++;
                }
            }
            i_b--;
        }
        // reset to false
        wall_found_l = false;
        wall_found_r = false;
    }
    return correct_count/count;
}

float grid_util::wall_accuracy(std::vector<std::vector<int>> grid_pred) {
    float correct_count = 0.0;
    float count = 0.0;
    for (int i=0; i<this->grid.size(); i++) {
        for (int j=0; j<grid[0].size(); j++) {
            // count += (this->grid[i][j] == 1);
            // correct_count += (grid_pred[i][j]==1);
            if (this->min_y <= j && j < this->max_y) {
                // if grid is 1 and it's beyong the min/max range
                count += (this->grid[i][j]==1 
                    && (i <= this->x_range[j-this->min_y][0]
                    || i >= this->x_range[j-this->min_y][1])) ? 1: 0;
                correct_count += (grid_pred[i][j]==1 
                    && (i <= this->x_range[j-this->min_y][0]
                    || i >= this->x_range[j-this->min_y][1])) ? 1: 0;
            }
            else {
                count += (this->grid[i][j]==1);
                correct_count += (grid_pred[i][j]==1);
            }

            // correct_count += (grid_pred[i][j]==1);
        }
    }
    // std::cout << "count: " << count << std::endl;
    // std::cout << "correct_count: " << correct_count << std::endl;
    return correct_count/count;
}