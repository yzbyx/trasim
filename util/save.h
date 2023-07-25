//
// Created by yzbyx on 2023/7/25.
//

#ifndef TRASIM_SAVE_H
#define TRASIM_SAVE_H


#include <string>
#include <map>
#include "../core/Constants.h"


void save_data_to_txt(const std::string &path_, const std::map<C_Info, std::vector<double>> & data);


#endif //TRASIM_SAVE_H
