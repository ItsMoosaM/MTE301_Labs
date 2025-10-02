//Draw objects in the environment

#include <SFML/Graphics.hpp>
#include <vector>
#include "utils.h"

#ifndef RENDER
#define RENDER

sf::RectangleShape draw_object(int, int, int, int);

void render_window(
    std::vector<std::vector<int>> &, 
    std::vector<Object *> &,
    Object, 
    int, 
    int,
    int);

void render_grid(
    Object, 
    std::vector<std::vector<int>>, 
    std::vector<std::vector<int>>, 
    int, int, int, int, int);

#endif