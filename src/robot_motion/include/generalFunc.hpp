#include "def.h"
#include "math.h"

#define wheel_radius 0.5
#define base_lenght 0.2
#define wheel_ratio 48
#define rpm_max 100

int mapPWM(double x, int in_min, int in_max, int minOut, int maxOut)
{
    return (x - in_min) * (maxOut - minOut) / (in_max - in_min) + minOut;
}

// Inverskinematic equation diffrential wheel
void inversKinematic(motor &output, double v = 0, double w = 0)
{
    output.w1 = (v - (base_lenght / 2.0) * w) / (2 * M_PI * wheel_radius);
    output.w2 = (v + (base_lenght / 2.0) * w) / (2 * M_PI * wheel_radius);
}

void rpsToPwm(pwmMotor &out, double rps1, double rps2)
{
    double rpm1 = wheel_ratio * (rps1 * (60 / (2 * M_PI)));
    double rpm2 = wheel_ratio * (rps2 * (60 / (2 * M_PI)));
    IC(rpm1, rpm2);
    if (rpm1 > 0)
        out.left = mapPWM(rpm1, 5, rpm_max, 60, 255);
    else if(rpm1 < 0)
        out.left = mapPWM(rpm1, -rpm_max, -5, -255, -60);
    else
        out.right = 0;

    if (rpm2 > 0)
        out.right = mapPWM(rpm2, 5, rpm_max, 60, 255);
    else if(rpm2 < 0)
        out.right = mapPWM(rpm2, -rpm_max, -5, -255, -60);
    else
        out.right = 0;
    //   if (rpm1 > 0)
    //     out.left = mapPWM(rpm1, 0, rpm_max, 0, 255);
    // else
    //     out.left = mapPWM(rpm1, -rpm_max, 0, -255, 0);

    // if (rpm2 > 0)
    //     out.right = mapPWM(rpm2, 0, rpm_max,0, 255);
    // else
    //     out.right = mapPWM(rpm2, -rpm_max, 0, -255, 0);

    // out.left = (255 / rpm_max) * rpm1;
    // out.right = (255 / rpm_max) * rpm2;
    // double rpm1 = (21.26 * rps1 * rps1 * rps1 * rps1 * rps1) -
    //               (92.68 * rps1 * rps1 * rps1 * rps1) + (151 * rps1 * rps1 * rps1)
    //               -(102.5 * rps1 * rps1) + (49.68 * rps1) + 5.047;

    // double rpm2 = (21.26 * rps2 * rps2 * rps2 * rps2 * rps2) -
    //               (92.68 * rps2 * rps2 * rps2 * rps2) + (151 * rps2 * rps2 * rps2)
    //               -(102.5 * rps2 * rps2) + (49.68 * rps2) + 5.047;

    // out.left = rpm1;
    // out.right = rpm2;

    // if(out.left > 0 && out.left < 90)
    //     out.left = 90;
    // else if(out.left < 0 && out.left > -90)
    //     out.left = -90;
    // if(out.right > 0 && out.right < 90)
    //     out.right = 90;
    //  else if(out.right < 0 && out.right > -90)
    //     out.right = -90;
}
