//
// Created by yzbyx on 2023/6/23.
//

#ifndef TRASIM_OBSTACLE_H
#define TRASIM_OBSTACLE_H

#include <SFML/Graphics/RectangleShape.hpp>
#include "Constants.h"

class Obstacle {
public:
    double x;
    double v;
    double a;
    Color color;

    double length;
    double width;
    VType type;

    explicit Obstacle(VType type_);
    virtual ~Obstacle();

    sf::RectangleShape * car_shape;
};


#endif //TRASIM_OBSTACLE_H
