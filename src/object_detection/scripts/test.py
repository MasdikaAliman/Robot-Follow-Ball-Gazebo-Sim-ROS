#!/usr/bin/env python3

import math


def f(x):
    return 5 * x*x*x + 5 * x*x - 20 * x + 8.5

# def ()

def yn(yn_1, k1 , k2, k3 ,k4):
    return yn_1 + (0.5 / 6) * (k1 + 2 *k2 + 2* k3 + k4)

print(f(2.75))
print(yn(15.418 ,  28.5, 45.76, 67.875, 95.296))