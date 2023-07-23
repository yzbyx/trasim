#include <iostream>
#include "core/Obstacle.h"

int main() {
    Obstacle obstacle(VType::OBSTACLE);
    std::cout << obstacle.x;
    return 0;
}
