//
// Created by yzbyx on 2023/7/25.
//

#include <fstream>
#include "save.h"

void save_data_to_txt(const std::string &path_, const std::map<C_Info, std::vector<double>>& data) {
    std::ofstream file(path_, std::ios::out);
    std::vector<std::vector<double>*> pure_data = {};

    // 打印header
    for (const auto& pair : data) {
        file << ALL_C_INFO_2_STRING.at(pair.first) << "\t";
        pure_data.push_back(const_cast<std::vector<double>*>(&pair.second));
    }
    // 删除每行末尾的最后一个制表符
    file.seekp(-1, std::ios_base::cur);
    file << std::endl;

    // 逐行打印数据
    size_t num_rows = 0;
    for (const auto& vec_ptr : pure_data) {
        num_rows = std::max(num_rows, vec_ptr->size());
    }

    for (size_t row = 0; row < num_rows; ++row) {
        for (const auto& vec_ptr : pure_data) {
            if (row < vec_ptr->size()) {
                file << (*vec_ptr)[row] << "\t";
            } else {
                file << "N/A\t"; // 如果行不足，用 N/A 填充
            }
        }
        // 删除每行末尾的最后一个制表符
        file.seekp(-1, std::ios_base::cur);
        file << std::endl;
    }

    file.close();
}
