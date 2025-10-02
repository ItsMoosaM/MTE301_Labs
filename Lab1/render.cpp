#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <chrono>

// #include "drawobjects.h"
#include "utils.h"

// Text colours
const std::string RED = "\033[31m";   // Red text
const std::string GREEN = "\033[32m"; // Green text
const std::string RESET = "\033[0m";  // Reset to default

sf::RectangleShape draw_object(int x, int y, int width, int height) {
    sf::RectangleShape obj(sf::Vector2f(width, height));
    sf::Vector2f linePosition(x, y);
    obj.setPosition(linePosition);
    return obj;  
}

// draw
void draw(
    sf::RenderWindow &window,
    sf::CircleShape &robot_draw,
    sf::Vector2f &robotPosition,
    std::vector<sf::RectangleShape> &objects_draw,
    std::vector<std::vector<int>> &robot_pos,
    sf::RectangleShape &goal_draw,
    std::vector<sf::RectangleShape>::iterator &i,
    bool &isPaused,
    bool &succeed,
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
    window.clear();

    // Drawing operations
    robot_draw.setPosition(robotPosition);
    window.draw(robot_draw);
    window.draw(goal_draw);
    // window.draw(line);
    for (i = objects_draw.begin(); i != objects_draw.end(); ++i){
        window.draw(*i);
    }

    if ((count >= robot_pos.size()-1) && succeed) {
        std::cout << GREEN << "Success! Goal reached!" << RESET << std::endl;
        window.close();
    }
    if ((count >= robot_pos.size()-1) && !succeed) {
        std::cout << RED << "Failure! Collision!" << RESET << std::endl;
        window.close();
    }

    // end the current frame
    window.display();

    // increment only if not paused
    if (!isPaused) {
        count++;
    }    
}

void render_window(
    std::vector<std::vector<int>> robot_pos,
    std::vector<Object> objects, 
    Object robot, 
    Object goal, 
    int width, 
    int height,
    bool succeed)
{
    int del_x, del_y, vel_x, vel_y;

    //==========Setup==========

    // get the screen size to center the window
    int desktop_width = sf::VideoMode::getDesktopMode().width;
    int desktop_height = sf::VideoMode::getDesktopMode().height;
    int middle_x = (desktop_width/2) - width/2;
    int middle_y = (desktop_height/2) - width/2;    

    // create the window
    sf::RenderWindow window(sf::VideoMode(width, height), "MTE301 Lab 1");
    window.setFramerateLimit(60);
    window.setPosition(sf::Vector2i(middle_x, middle_y));

    // create the robot
    sf::CircleShape robot_draw(robot.width/2);
    robot_draw.setFillColor(sf::Color::Blue);
    sf::Vector2f robotPosition(robot.x, robot.y);
    robot_draw.setPosition(robotPosition);

    sf::RectangleShape goal_draw(sf::Vector2f(goal.width, goal.height));
    sf::Vector2f goalPosition(goal.x, goal.y);
    
    goal_draw.setPosition(goalPosition);
    goal_draw.setFillColor(sf::Color::Green);    

    std::vector<sf::RectangleShape> objects_draw;
    // Now spawn the objects in SFML. 
    for (int i = 0; i < objects.size(); i++) {
        objects_draw.push_back(draw_object(objects[i].x, objects[i].y, objects[i].width, objects[i].height));
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
                window.close();
            }
        }

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
                    window.setTitle("MTE301 Lab 1");
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
            if (elapsed_left > 10) {
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
            if (elapsed_right > 10) {
                if (isPaused && (count <= robot_pos.size())) {
                    // std::cout << "Moving back " << left_count << " timesteps" << std::endl;
                    right_count = 0;
                    count++;
                }
            }
        }

        draw(window, robot_draw, robotPosition, objects_draw, robot_pos, goal_draw, i, isPaused, succeed, count);
        
    }
}
