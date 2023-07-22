//
// Created by yzbyx on 2023/7/21.
//

#ifndef TRASIM_BASE_H
#define TRASIM_BASE_H

#include <iostream>

class base {
public:
    explicit base(int i);

    int num;

    virtual void test();
};


#endif //TRASIM_BASE_H
