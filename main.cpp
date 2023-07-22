#include <iostream>
#include "core/Obstacle.h"

int main() {
    Obstacle obstacle(VType::OBSTACLE);
    std::printf("%s", reinterpret_cast<const char *>(obstacle.color));
    // 使用obstacle对象进行操作
    return 0;
}
