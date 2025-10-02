#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <chrono>
#include <map>

// #include "drawobjects.h"
#include "utils.h"

// Text colours
const std::string RED = "\033[31m";   // Red text
const std::string GREEN = "\033[32m"; // Green text
const std::string RESET = "\033[0m";  // Reset to default

// sf::RectangleShape draw_object(int x, int y, int width, int height, int val) {
//     sf::RectangleShape obj(sf::Vector2f(width, height));
//     sf::Vector2f linePosition(x, y);
//     obj.setPosition(linePosition);
//     switch (val) {
//         case 1:
//         default:
//             obj.setFillColor(sf::Color::Black);
//             break;
//         case 2:
//             obj.setFillColor(sf::Color::Green);
//             break;
//     };
    
//     return obj;  
// }

sf::RectangleShape draw_object(Object *obj_) {
    sf::RectangleShape obj(sf::Vector2f(obj_->width, obj_->height));
    sf::Vector2f linePosition(obj_->x, obj_->y);
    obj.setPosition(linePosition);
    switch (obj_->val) {
        case 1:
        default:
            obj.setFillColor(sf::Color::Black);
            break;
        case 2:
            obj.setFillColor(sf::Color::Green);
            break;
    };
    
    return obj;  
}

sf::RectangleShape draw_angled_object(int x, int y, int width, int height, float angle) {
    sf::RectangleShape obj(sf::Vector2f(width, height));
    obj.setOrigin(width/2.f, 0);
    obj.setRotation(angle);
    sf::Vector2f Position(x, y);
    obj.setPosition(Position);
    obj.setFillColor(sf::Color::Black);
    return obj;  
}

// draw
void draw(
    sf::RenderWindow &window,
    sf::CircleShape &robot_draw,
    sf::Vector2f &robotPosition,
    std::vector<sf::RectangleShape> &objects_draw,
    std::vector<std::vector<int>> &robot_pos,
    std::vector<sf::RectangleShape>::iterator &i,
    bool &isPaused,
    int &count
) {
    if (count < robot_pos.size() && robot_pos[count].size() >= 2) {
        robotPosition.x = robot_pos[count][0]; // Safe access
        robotPosition.y = robot_pos[count][1]; // Safe access
    } else {
        if (count < 200) {
            std::cerr << "Error: Accessing out of bounds for robot_pos at count: " << count << std::endl;
            // Handle error appropriately (e.g., skip iteration, set defaults, etc.)
        }
    }

    // clear the window
    window.clear(sf::Color::White);

    // Drawing operations
    robot_draw.setPosition(robotPosition);
    window.draw(robot_draw);
    // window.draw(line);
    for (i = objects_draw.begin(); i != objects_draw.end(); ++i){
        window.draw(*i);
    }

    if (count >= robot_pos.size()-1) {
        std::cout << "==========Ground-truth window terminated==========" << std::endl;
        window.close();
    }

    // end the current frame
    window.display();

    // increment only if not paused
    if (!isPaused) {
        count++;
    }
}

void check_keypress (
    std::chrono::steady_clock::time_point &prev_pause,
    int64_t &elapsed_pause,
    std::chrono::steady_clock::time_point &prev_left, std::chrono::steady_clock::time_point &prev_right,
    int64_t &elapsed_left, int64_t &elapsed_right,
    int &count, int &left_count, int &right_count, bool &isPaused,
    sf::RenderWindow &window,
    std::vector<std::vector<int>> &robot_pos,
    std::string &title
) {

    // check for pausing. press p to pause
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::P)) {
        elapsed_pause = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::steady_clock::now() - prev_pause).count();
        prev_pause = std::chrono::steady_clock::now();
        if (elapsed_pause > 200) {
            isPaused = !isPaused;
            if (isPaused) {
                window.setTitle("PAUSED. Press p to unpause. Press left/right keys to move back/forward in time.");
                // std::cout << "PAUSED. Press p to unpause" << std::endl;
            }
            else {
                window.setTitle(title);
                // std::cout << "UNPAUSED" << std::endl;
            }
        }
    }

    // check for left key for rewind
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
        elapsed_left = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::steady_clock::now() - prev_left).count();
        prev_left = std::chrono::steady_clock::now();
        left_count++;
        // originally 10, lowered to account higher fps
        if (elapsed_left > 3) {
            if (isPaused && (count >= 1)) {
                // std::cout << "Moving back " << left_count << " timesteps" << std::endl;
                left_count = 0;
                count--;
            }
        }
    }

    // check for right key for forward
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
        elapsed_right = std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::steady_clock::now() - prev_right).count();
        prev_right = std::chrono::steady_clock::now();
        right_count++;
        if (elapsed_right > 3) {
            if (isPaused && (count <= robot_pos.size())) {
                // std::cout << "Moving back " << left_count << " timesteps" << std::endl;
                right_count = 0;
                count++;
            }
        }
    }
}

void render_window(
    std::vector<std::vector<int>> &robot_pos,
    std::vector<Object *> &objects,
    Object robot, 
    int width, 
    int height,
    int render_speed)
{
    int del_x, del_y, vel_x, vel_y;

    //==========SFML==========

    // retrieve screen resolution to center the window
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();

    int top_left_x = (desktop.width/2) - (width/2);
    int top_left_y = (desktop.height/2) - (height/2);

    // create the window
    std::string title = "MTE301 Lab 5 Ground Truth Map at " + std::to_string(render_speed) + "x Speed";
    sf::RenderWindow window(sf::VideoMode(width, height), title);
    window.setFramerateLimit(60*render_speed);
    window.setPosition(sf::Vector2i(top_left_x, top_left_y));
    window.clear(sf::Color::White);

    // create the robot
    sf::CircleShape robot_draw(robot.width/2);
    robot_draw.setFillColor(sf::Color::Blue);
    sf::Vector2f robotPosition(robot.x, robot.y);
    robot_draw.setPosition(robotPosition);
  
    std::vector<sf::RectangleShape> objects_draw;
    // is it an angled wall (Wall) or a regular wall (Object)?
    // bool is_angled = objects[0]->type() == 1? true: false;
    bool is_angled;
    for (int i = 0; i < objects.size(); i++) {
        is_angled = (dynamic_cast<Wall*>(objects[i]))? true: false;
        if (!is_angled) {
            objects_draw.push_back(draw_object(objects[i]));
            // if not last entry, draw with value 1 (wall/obstacle)
            // if (!(i==objects.size()-1)) {
            //     // objects_draw.push_back(draw_object(
            //     //     objects[i]->x, objects[i]->y, objects[i]->width, objects[i]->height, 1));
            //     objects_draw.push_back(draw_object(objects[i]));
            // }
            // // if last entry, draw with value 2 (goal)
            // else {
            //     objects_draw.push_back(draw_object(objects[i]));
            //     // objects_draw.push_back(draw_object(
            //     //     objects[i]->x, objects[i]->y, objects[i]->width, objects[i]->height, 2));
            // }

        }
        else {
            // std::cout << "Wall!\n";
            Wall* wall = dynamic_cast<Wall*>(objects[i]);
            objects_draw.push_back(draw_angled_object(
                wall->x, wall->y, wall->width, wall->height, wall->angle));
        }
    }

    std::vector<sf::RectangleShape>::iterator i;
    int count = 0;
    bool isPaused{false}; // State to track whether the game is paused
    int left_count{0}; // counter for how many times left was pressed
    int right_count{0}; // counter for how many times right was pressed

    // timers for button press events
    auto prev_pause = std::chrono::steady_clock::now();
    auto elapsed_pause = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::steady_clock::now() - prev_pause).count();
    auto prev_left = std::chrono::steady_clock::now();
    auto elapsed_left = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::steady_clock::now() - prev_left).count();
    auto prev_right = std::chrono::steady_clock::now();
    auto elapsed_right = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::steady_clock::now() - prev_right).count();

    //==========Main loop==========
    while (window.isOpen())
    {

        // check all the window's events that were triggered since the last iteration of the loop
        sf::Event event;
        while (window.pollEvent(event))
        {
            // "close requested" event: we close the window
            if(event.type == sf::Event::Closed){
                std::cout << "==========Ground-truth window terminated==========" << std::endl;
                window.close();
            }
        }

        check_keypress (prev_pause, elapsed_pause, 
            prev_left, prev_right, 
            elapsed_left, elapsed_right,
            count, left_count,right_count, 
            isPaused, window, robot_pos, title);

        draw(window, robot_draw, robotPosition, objects_draw, robot_pos, i, isPaused, count);
        
    }
}

void draw_grid(
    sf::RenderWindow &window,
    std::vector<std::vector<int>> &robot_pos,
    std::vector<std::vector<int>> &r_grid,
    sf::CircleShape &robot,
    sf::CircleShape &lidar,
    sf::Image &freeSpaceImage,
    sf::Texture &freeSpaceTexture,
    std::map<int, sf::Color> &grid_cell,
    sf::Vector2f &robotPosition, sf::Vector2f &prevRobotPos,
    int &x_min, int &x_max, int &y_min, int &y_max, int &count, 
    int &radius, int &LIDAR_RADIUS, int &width, int &height,
    bool &isPaused) {

    if (count < robot_pos.size() && robot_pos[count].size() >= 2) {
        // add radius to account for the offset between corner and center
        robotPosition.x = robot_pos[count][0]+radius; // Safe access
        robotPosition.y = robot_pos[count][1]+radius; // Safe access
    } else {
        if (count < 200) {
            std::cerr << "Error: Accessing out of bounds for robot_pos at count: " << count << std::endl;
            // Handle error appropriately (e.g., skip iteration, set defaults, etc.)
        }
    }

    // drawing operations
    robot.setPosition(robotPosition);
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // update the portion of the screen the robot left behind
    if (robotPosition != prevRobotPos) {

        x_min = std::max(0, static_cast<int>(prevRobotPos.x - LIDAR_RADIUS));
        x_max = std::min(width - 1, static_cast<int>(prevRobotPos.x + LIDAR_RADIUS));
        y_min = std::max(0, static_cast<int>(prevRobotPos.y - LIDAR_RADIUS));
        y_max = std::min(height - 1, static_cast<int>(prevRobotPos.y + LIDAR_RADIUS));

        freeSpaceImage.create(2*LIDAR_RADIUS, 2*LIDAR_RADIUS, sf::Color::Transparent);
        
        // Loop over the bounding box and mark pixels as white if they are within the LiDAR radius
        for (int x = x_min; x < x_max; ++x) {
            for (int y = y_min; y < y_max; ++y) {
                // Check if the point is within the LiDAR radius
                if ((x - prevRobotPos.x) * (x - prevRobotPos.x) +
                    (y - prevRobotPos.y) * (y - prevRobotPos.y) <= LIDAR_RADIUS * LIDAR_RADIUS) {
                        // freeSpaceImage.setPixel(x - x_min, y - y_min, sf::Color::White); // Mark as free space (white)
                        freeSpaceImage.setPixel(x - x_min, y - y_min, grid_cell[r_grid[x][y]]);
                }
            }
        }

        // convert the Image to a Texture
        freeSpaceTexture.loadFromImage(freeSpaceImage);

        // convert the Texture to a Sprite
        sf::Sprite freeSpaceSprite(freeSpaceTexture);
        // this is top-left position. Subtract 10 to account for LiDAR range
        freeSpaceSprite.setPosition(x_min, y_min);

        // Draw the free space onto the window
        window.draw(freeSpaceSprite);

        // Update the previous position
        prevRobotPos = robotPosition;

    }

    // draw the robot
    window.draw(robot);

    window.display();

    if (count == robot_pos.size()-1) {
        std::cout << "==========Occupancy grid mapping stopped after " << robot_pos.size() << " iterations==========" << std::endl;
    }

    // increment only if not paused
    if (!isPaused) {
        count++;
    }

}

void render_grid(Object robot_,
    std::vector<std::vector<int>> robot_pos, 
    std::vector<std::vector<int>> r_grid,
    int width, int height, int radius, int l_radius, int render_speed) {

    // retrieve screen resolution to center the window
    sf::VideoMode desktop = sf::VideoMode::getDesktopMode();

    int top_left_x = (desktop.width/2) - (width/2);
    int top_left_y = (desktop.height/2) - (height/2);

    // create the window
    std::string title = "MTE301 Lab 5 Occupancy Grid Map at " + std::to_string(render_speed) + "x Speed";
    sf::RenderWindow window(sf::VideoMode(width, height), title);
    window.setFramerateLimit(60*render_speed);
    window.setPosition(sf::Vector2i(top_left_x, top_left_y));
    window.clear(sf::Color(128, 128, 128));
    window.display();

    // map for key-value pair between occupancy and grid cell colour
    std::map<int, sf::Color> grid_cell = {{0, sf::Color::White}, 
                                        {1, sf::Color::Black}, 
                                        {-1, sf::Color(128, 128, 128)},
                                        {2, sf::Color(0, 255, 0)}};

    // Robot circle shape
    sf::CircleShape robot(radius);
    robot.setFillColor(sf::Color::Blue); // Green circle for the robot
    robot.setOrigin(radius, radius); // Set origin to the center
    robot.setPosition(robot_.x+radius, robot_.y+radius); // Initially place the robot at the center
    sf::Vector2f robotPosition(robot_.x, robot_.y);

    // LiDAR range circle
    sf::CircleShape lidar(l_radius);
    lidar.setFillColor(sf::Color(255, 0, 0, 128)); // Semi-transparent red (128 alpha)
    lidar.setOrigin(l_radius, l_radius); // Set origin to the center
    lidar.setPosition(robot.getPosition()); // Initially place the LiDAR circle on the robot

    // previous robot position to track where it left
    sf::Vector2f prevRobotPos = robot.getPosition();
    int count = 0;

    // variables for handling key events
    bool isPaused{false}; // State to track whether the game is paused
    int left_count{0}; // counter for how many times left was pressed
    int right_count{0}; // counter for how many times right was pressed

    // timers for button press events
    auto prev_pause = std::chrono::steady_clock::now();
    auto elapsed_pause = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::steady_clock::now() - prev_pause).count();
    auto prev_left = std::chrono::steady_clock::now();
    auto elapsed_left = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::steady_clock::now() - prev_left).count();
    auto prev_right = std::chrono::steady_clock::now();
    auto elapsed_right = std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::steady_clock::now() - prev_right).count();

    // create an sf::Image to manually set pixels
    sf::Image freeSpaceImage;
    // convert the Image to a Texture
    sf::Texture freeSpaceTexture;

    // define min/max for bounding box of lidar circle in previous frame
    int x_min, x_max, y_min, y_max;
    
    // main loop
    while (window.isOpen()) {
        // count++;
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        check_keypress (prev_pause, elapsed_pause, 
            prev_left, prev_right, 
            elapsed_left, elapsed_right,
            count, left_count,right_count, 
            isPaused, window, robot_pos, title);

        draw_grid(window, robot_pos, r_grid, robot, lidar, 
            freeSpaceImage, freeSpaceTexture,
            grid_cell,
            robotPosition, prevRobotPos,
            x_min, x_max, y_min, y_max, count, 
            radius, l_radius, width, height, isPaused);

    }
}