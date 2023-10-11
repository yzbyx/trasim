//
// Created by yzbyx on 2023/9/19.
//

#ifndef TRASIM_NODE_H
#define TRASIM_NODE_H

# include <vector>
# include "../micro/LaneAbstract.h"

class Node {
public:
    Node() = default;
    std::vector<LaneAbstract*> next_lanes;

};


#endif //TRASIM_NODE_H
