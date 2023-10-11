//
// Created by yzbyx on 2023/7/25.
//

#include <fstream>
#include <sstream>
#include "saver.h"
#include "print.h"

void save_data_to_txt(const std::string &path_, const std::map<C_Info, std::vector<double>>& data) {
    std::ofstream file(path_, std::ios::out);
    std::vector<std::vector<double>*> pure_data = {};

    // 打印header
    std::vector<std::string> headers;
    size_t num_rows = -1;
    for (const auto& pair : data) {
        file << ALL_C_INFO_2_STRING.at(pair.first) << "\t";
//        headers.push_back(ALL_C_INFO_2_STRING.at(pair.first));
        pure_data.push_back(const_cast<std::vector<double>*>(&pair.second));
        if (num_rows == -1) {
            num_rows = pair.second.size();
        }
    }
    file << "\n";

    // 逐行打印数据

    for (size_t row = 0; row < num_rows; ++row) {
        for (const auto& vec_ptr : pure_data) {
            file << (*vec_ptr)[row] << "\t";
//            temp.push_back((*vec_ptr)[row]);
        }
        file << "\n";
    }

    file.close();
}
