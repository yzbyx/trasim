//
// Created by yzbyx on 2023/8/1.
//

#ifndef TRASIM_PRINT_H
#define TRASIM_PRINT_H

#include <string>
#include <vector>

std::string add_sep(const std::vector<double>& data, const std::string& sep, size_t length=0);

std::string add_sep(const std::vector<std::string>& data, const std::string& sep, size_t length=0);

#endif //TRASIM_PRINT_H
