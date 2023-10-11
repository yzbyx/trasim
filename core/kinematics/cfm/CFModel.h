//
// Created by yzbyx on 2023/7/9.
//
#ifndef TRASIM_CFMODEL_H
#define TRASIM_CFMODEL_H

#include <random>
#include "../Model.h"
#include "../../Constants.h"

#define DEFAULT_EXPECT_DEC 3.0
#define DEFAULT_EXPECT_ACC 3.0
#define DEFAULT_EXPECT_SPEED 30.0

class Vehicle;

class CFModel : public Model {
public:
    CFM name;
    std::mt19937 *random;

    ~CFModel() override;

    explicit CFModel(std::shared_ptr<Vehicle> vehicle_);

    virtual double step(int index) = 0;

    virtual double get_expect_dec();

    virtual double get_expect_acc();

    virtual double get_expect_speed();

    /**
     * 通过平衡态速度计算三参数
     * @param speed
     * @param dhw
     * @param v_length
     * @return {k, q, v}
     */
    virtual std::vector<double> equilibrium_state(double speed, double dhw, double v_length);

    virtual double basic_diagram_k_to_q(double dhw, double car_length, double speed_limit);

    [[nodiscard]] virtual double get_jam_density(double v_length) const;

    void get_qm(...);

    double get_speed_limit();
};

std::shared_ptr<CFModel> get_cf_model(const std::shared_ptr<Vehicle>& _driver, CFM name, const std::map<std::string, double> &param);

#endif //TRASIM_CFMODEL_H
