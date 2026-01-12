#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include "rules.h"
const double e_norm = 5;  // 误差模糊化范围
const double de_norm = 5; // 误差导数模糊化范围
namespace fuzzy_pid
{
    class FuzzyPID
    {
    public:
        FuzzyPID();
        double compute(double setpoint, double pv, double dt)
        {
            error = setpoint - pv;
            if (fabs(error) > 0.25)
            {
                integral = 0;
            }
            else
            {
                integral += error * dt;
            }
            error_dot = (error - prev_error) / dt;
            prev_error = error;
            membership.empty();
            // 模糊化输入
            Get_membership(error / e_norm, e_range);
            Get_membership(error_dot / de_norm, de_range);
            // 得到输入对应的隶属度函数
            // 下结合规则得到输出解模糊化
            delta_kd = 0;
            delta_ki = 0;
            delta_kp = 0;
            defuzz_method();
            kp += delta_kp;
            ki += delta_ki;
            kd += delta_kd;
            kp = fabs(kp) > kp_max ? kp * kp_max / fabs(kp) : kp;
            ki = fabs(ki) > ki_max ? ki * ki_max / fabs(ki) : ki;
            kd = fabs(kd) > kd_max ? kd * kd_max / fabs(kd) : kd;
            return kp * error + ki * integral + kd * error_dot;
        }

    private:
        double kp, ki, kd;
        double kp_max, ki_max, kd_max;
        double delta_kp, delta_ki, delta_kd;
        double delta_kp_max, delta_ki_max, delta_kd_max;
        std::vector<double> membership;
        std::vector<int> index;
        double error;
        double prev_error;
        double error_dot;
        double learn_rate; // 学习率
        double integral;
        // 这里采用重心法解模糊
        void defuzz_method();
        // 输入模糊化
        void Get_membership(double input, float *range);
        // 三角隶属度函数
        double trimf(double input, double min, double max);
    };
} // namespace fuzzy_pid
