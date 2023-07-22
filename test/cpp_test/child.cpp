//
// Created by yzbyx on 2023/7/21.
//

#include "child.h"

child::child(int i) : base(i) {
    c_num = i + 1;
    test();
}

void child::test() {
    std::cout << "child::test" << std::endl;
}
