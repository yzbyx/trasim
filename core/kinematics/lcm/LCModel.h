//
// Created by yzbyx on 2023/7/20.
//

#ifndef TRASIM_LCMODEL_H
#define TRASIM_LCMODEL_H

#include "../Model.h"
#include "LCModel_KK.h"

class LCModel : public Model{
public:
    explicit LCModel(Vehicle* vehicle);
    ~LCModel() override = default;
    virtual std::map<std::string, double> base_cal() = 0;
    virtual std::map<std::string, double> on_ramp_cal() = 0;
    LCM name;
    std::mt19937 * random;
protected:
    double last_lc_time;
};

LCModel* get_lc_model(Vehicle* _driver, LCM name, const std::map<std::string, double>& param);


#endif //TRASIM_LCMODEL_H
