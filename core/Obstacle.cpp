//
// Created by yzbyx on 2023/6/23.
//

#include "Obstacle.h"
#include "Constants.h"

Obstacle::Obstacle(VType type_) {
    x = 0;
    v = 0;
    a = 0;
    color = Color::RED;

    length = 5.0;
    width = 1.8;
    type = type_;

    car_shape = nullptr;
}

Obstacle::~Obstacle() = default;
