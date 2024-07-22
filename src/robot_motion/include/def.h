#ifndef DEF_H
#define DEF_H

struct motor
{
    motor() : w1(0), w2(0) {}
    motor(double w1_, double w2_) : w1(w1_), w2(w2_) {}
    double w1, w2;
};

struct pwmMotor
{
    pwmMotor() : left(0), right(0) {}
    pwmMotor(int left_, int right_) : left(left_), right(right_) {}
    int left, right;
};


#endif