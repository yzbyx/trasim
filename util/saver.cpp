//
// Created by yzbyx on 2023/7/25.
//

#include <fstream>
#include "saver.h"
#include "print.h"
#include <Python.h>


void save_data_to_txt(const std::string &path_, const std::map<C_Info, std::vector<double>>& data) {
    std::ofstream file(path_, std::ios::out);
    std::vector<std::vector<double>*> pure_data = {};

    // 打印header
    std::vector<std::string> headers;
    for (const auto& pair : data) {
        headers.push_back(ALL_C_INFO_2_STRING.at(pair.first));
        pure_data.push_back(const_cast<std::vector<double>*>(&pair.second));
    }
    file << add_sep(headers, "\t") + "\n";

    // 逐行打印数据
    size_t num_rows = pure_data[0]->size();

    for (size_t row = 0; row < num_rows; ++row) {
        std::vector<double> temp;
        for (const auto& vec_ptr : pure_data) {
            temp.push_back((*vec_ptr)[row]);
        }
        file << add_sep(temp, "\t") + "\n";
    }

    file.close();
}
