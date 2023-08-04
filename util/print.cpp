//
// Created by yzbyx on 2023/8/1.
//

#include "print.h"

std::string add_sep(const std::vector<double>& data, const std::string& sep, size_t length) {
    if (length == 0) {
        length = data.size();
    }
    std::string s;
    for (int i = 0; i < length; ++i) {
        s.append(std::to_string(data[i]));
        if (i != length - 1) {
            s.append(sep);
        }
    }
    return s;
}

std::string add_sep(const std::vector<std::string>& data, const std::string& sep, size_t length) {
    if (length == 0) {
        length = data.size();
    }
    std::string s;
    for (int i = 0; i < length; ++i) {
        s.append(data[i]);
        if (i != length - 1) {
            s.append(sep);
        }
    }
    return s;
}
