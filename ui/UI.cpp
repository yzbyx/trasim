//
// Created by yzbyx on 2023/7/25.
//

#include <thread>
#include "UI.h"
#include "../core/frame/micro/LaneAbstract.h"
#include "../core/frame/micro/Road.h"


UI::UI(Road* frame_abstract) : frame(frame_abstract), frame_rate(-1),
                                                     width_base(1000), width_scale(1.5),
                                                     height_scale(2), single_height(20),
                                                     base_line_factor(2), screen_width(0),
                                                     screen_height(0), screen(nullptr), clock(nullptr) {
}

void UI::ui_init(const std::string& caption, int frame_rate_) {
    this->frame_rate = frame_rate_;
    screen_height = static_cast<int>((frame->lane_length / 1000 + base_line_factor) * single_height * height_scale);
    screen_width = static_cast<int>(width_base * width_scale);
    screen = new sf::RenderWindow(sf::VideoMode(screen_width, screen_height), caption);
    clock = std::make_unique<sf::Clock>();

    lane_list = frame->lane_list;

    ui_update();

//    std::thread thread_ui([&]() { ui_thread(*this); });

}

std::mutex uiMutex; // 定义互斥锁

void ui_thread(UI & ui) {
    int step = -1;
    while (ui.screen->isOpen()) {
        sf::Event event;
        while (ui.screen->pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                ui.screen->close();
        }
        // 使用互斥锁保护对ui对象的访问
        std::lock_guard<std::mutex> lock(uiMutex);

        if (ui.frame_rate > 0) {
            sf::Time elapsed = ui.clock->restart();
            sf::sleep(sf::milliseconds(static_cast<int>((1000. / ui.frame_rate) - elapsed.asMilliseconds())));
        } else {
            sf::sleep(sf::milliseconds(10));
        }
        if (step != ui.frame->step_) {
            ui.ui_update();
            ui.screen->display();
            step = ui.frame->step_;
        }
    }
}

void UI::ui_update() {
    std::lock_guard<std::mutex> lock(uiMutex);

    screen->clear(sf::Color::Black);

    int lane_width = 5;
    int start_y = base_line_factor * single_height;

    sf::Font font;
    font.loadFromFile(R"(E:\CProject\trasim\ui\times.ttf)");
    sf::Text text("steps: " + std::to_string(frame->step_), font, 20);
    text.setFillColor(sf::Color::White);
    screen->draw(text);

    int row_total = static_cast<int>((1.0 * frame->lane_length) / width_base) + 1;
    for (int row = 0; row < row_total; ++row) {
        int pos_y = start_y + row * single_height;
        sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(0, pos_y * height_scale), sf::Color::White),
                sf::Vertex(sf::Vector2f(width_base * width_scale, pos_y * height_scale), sf::Color::White)
        };
        screen->draw(line, 2, sf::Lines);
    }

    for (const auto& lane : lane_list) {
        for (size_t i = 0; i < lane->car_list.size(); ++i) {
            Vehicle* car = lane->car_list[i];
            int row = static_cast<int>(car->x / width_base);
            int offset = start_y + lane->index * lane_width + row * single_height;
            int pos_y = static_cast<int>((offset + lane_width / 2. - car->width / 2) * height_scale);
            int pos_x = static_cast<int>((car->x - row * width_base) * width_scale);

            if (car->car_shape == nullptr) {
                car->car_shape = new sf::RectangleShape(sf::Vector2f(car->length * width_scale,
                                                                               car->width * height_scale));
                std::vector<int> temp = COLOR_2_RGB.at(lane->car_list[i]->color);
                car->car_shape->setFillColor(sf::Color(temp[0], temp[1], temp[2]));
            }
//            std::cout << std::to_string(static_cast<int>(lane->car_list[i]->color)) << std::endl;

//            car_shape.setFillColor(sf::Color::Yellow);
            car->car_shape->setPosition(pos_x, pos_y);
            screen->draw(*car->car_shape);
        }
    }
    screen->display();
}

UI::~UI() {
    delete screen;
}
