//Draw objects in the environment

#include <SFML/Graphics.hpp>
#include <vector>
#include "utils.h"

#ifndef RENDER
#define RENDER

sf::RectangleShape draw_object(int, int, int, int);

void render_window(
    std::vector<std::vector<int>>, 
    std::vector<Object>, 
    Object, 
    Object, 
    int, 
    int, 
    bool);

#endif