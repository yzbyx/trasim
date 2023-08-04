# -*- coding = uft-8 -*-
# @Time : 2023-03-31 16:20
# @Author : yzbyx
# @File : data_container.py
# @Software : PyCharm
class Info:
    lane_add_num = "Lane Add Num"
    """车辆所在车道在该路段的添加次序，以0为开始依次累加"""
    id = "ID"
    """车辆ID"""
    step = "Step"
    """仿真步次"""
    time = "Time [s]"
    """仿真时间"""
    a = "Acceleration [m/s^2]"
    """加速度"""
    v = "Velocity [m/s]"
    """速度"""
    x = "Position [m]"
    """位置"""
    dhw = "Distance Headway [m]"
    """车头间距"""
    thw = "Time Headway [s]"
    """车头时距"""
    gap = "Gap [m]"
    """净间距"""
    dv = "Dv [m/s]"
    """前车与后车速度差"""
    cf_id = "cf model ID"
    """跟驰模型类别ID"""
    lc_id = "lc model ID"
    """换到模型类别ID"""
    car_type = "car type"
    """车辆类型"""

    safe_ttc = "ttc (s)"
    safe_tet = "tet"
    safe_tit = "tit (s)"
    safe_picud = "picud (m)"
    safe_picud_KK = "picud_KK (m)"
    """KK模型适用的picud计算方式（前车最大减速度等于当前车的最大减速度）"""

    @classmethod
    def get_all_info(cls):
        dict_ = Info.__dict__
        values = {}
        for key in dict_.keys():
            if isinstance(dict_[key], str) and key[:2] != "__":
                values.update({key: dict_[key]})
        return values


if __name__ == '__main__':
    print(Info.get_all_info())
