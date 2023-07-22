//
// Created by yzbyx on 2023/7/21.
//

#ifndef TRASIM_CHILD_H
#define TRASIM_CHILD_H


#include "base.h"

class child : public base{
public:
    explicit child(int i);

    int c_num;

    void test() override;
};


#endif //TRASIM_CHILD_H
