#pragma once

#include "circle.h"
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <vector>

typedef double real;

class CircleFit
{
    static Point calcMean(std::vector<Point>& pointcloud);

    static real calcSigma(std::vector<Point>& pointcloud, real center_x, real center_y, real radius);

    public:
    static bool pointcloudIsValid(std::vector<Point>& pointcloud);
    static Circle hyperFit(std::vector<Point>& pointcloud, int iter_max = 99);
}; // namespace CircleFit