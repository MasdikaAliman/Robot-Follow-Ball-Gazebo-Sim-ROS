#ifndef PID_H
#define PID_H
#include "math.h"
#include "boost/chrono.hpp"
#include "icecream.hpp"
class PID
{
public:
    PID(double max_linear = 10, double max_rotation = 3)
    {
        eIntegral = 0;
        e_prev = 0;
        eIntegral_rot = 0;
        e_prev_rot = 0;
        max_lin = max_linear;
        max_rot = max_rotation;
    }
    void setParam(double kp_, double ki_, double kd_)
    {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }
    void setParamRot(double kp, double ki, double kd)
    {
        kp_rot = kp;
        ki_rot = ki;
        kd_rot = kd;
    }

    void calc_PIDLinear(double setpoint, double actual, double &outputLinear)
    {
        start = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = start - end;
        end = start;

        time_lin = duration.count() / 1e6;

        error = setpoint - actual;
        IC(error);
        eIntegral += error;
        if (eIntegral > max_lin)
            eIntegral = max_lin;
        else if (eIntegral < -max_lin)
            eIntegral = -max_lin;
        // else

        IC(eIntegral);
        IC(kp, ki, kd);
        outputLinear = kp * error + ki * eIntegral + kd * (error - e_prev);
        IC(outputLinear);
        if (outputLinear > max_lin)
            outputLinear = max_lin;
        else if (outputLinear < -max_lin)
            outputLinear = -max_lin;
        IC(outputLinear);
        e_prev = error;
    }

    bool calc_PIDRot(double setPoint, double actual, double &output)
    {
        // start_rot = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::milli> duration = start_rot - end_rot;
        // end_rot = start_rot;

        // time_rot = duration.count() / 1e6;

        error_theta = setPoint - actual;

        if(error_theta > M_PI)
            error_theta -= 2 * M_PI;
        if(error_theta < -M_PI)
            error_theta += 2 * M_PI;
        IC(error_theta);

        if(fabs(error_theta)  < 3 * M_PI / 180.0)
        {
            output = 0;
            return 1;
        }
        // IC(error_theta);
        if (eIntegral_rot >= max_rot * M_PI / 180.0)
            eIntegral_rot = max_rot * M_PI / 180.0;
        else if (eIntegral_rot <= -max_rot * M_PI / 180.0)
            eIntegral_rot = -max_rot * M_PI / 180.0;
        else
            eIntegral_rot += error_theta;
        IC(eIntegral_rot, kp_rot, ki_rot, kd_rot);
        output = kp_rot * error_theta + ki_rot * eIntegral_rot + kd_rot * (error_theta - e_prev_rot);
        if (output >= max_rot * M_PI / 180.0)
            output = max_rot * M_PI / 180.0;
        else if (output <= -max_rot * M_PI / 180.0)
            output = -max_rot * M_PI / 180.0;

        IC(output);
        e_prev = error_theta;
        return true;
    }

private:
    double eIntegral, e_prev, eIntegral_rot, e_prev_rot;
    double kp, ki, kd;
    double kp_rot, ki_rot, kd_rot;
    double error;
    double error_theta;
    double time_lin, time_rot;
    double max_lin, max_rot;

    // time
    std::chrono::high_resolution_clock::time_point start, start_rot, end, end_rot;
};

#endif