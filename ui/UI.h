//
// Created by yzbyx on 2023/7/25.
//

#ifndef TRASIM_UI_H
#define TRASIM_UI_H


#include <string>
#include <vector>
#include <memory>
#include <SFML/Graphics.hpp>
#include <mutex>

class LaneAbstract; // 假设LaneAbstract类的定义在其他地方
class Road;

class UI {
public:
    ~UI();
    explicit UI(Road* frame_abstract);

    void ui_init(const std::string& caption, int frame_rate);
    void ui_update();

    sf::RenderWindow* screen;

    Road* frame;
    int frame_rate;
    int width_base;
    double width_scale;
    double height_scale;
    int single_height;
    int base_line_factor;
    int screen_width;
    int screen_height;
    std::vector<LaneAbstract*> lane_list; // 使用std::vector存储lane_list，要确保LaneAbstract类的定义在此之前
    std::unique_ptr<sf::Clock> clock;
};

void ui_thread(UI & ui);

#endif //TRASIM_UI_H
