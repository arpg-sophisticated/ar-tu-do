#include "circle_fit.h"

Point CircleFit::calcMean(std::vector<Point>& pointcloud)
{
    real sum_x = 0;
    real sum_y = 0;
    for (auto& point : pointcloud)
    {
        sum_x += point.x;
        sum_y += point.y;
    }
    return Point{ sum_x / pointcloud.size(), sum_y / pointcloud.size() };
}

real CircleFit::calcSigma(std::vector<Point>& pointcloud, real center_x, real center_y, real radius)
{
    real dx, dy, sum = 0;
    for (auto& point : pointcloud)
    {
        dx = point.x - center_x;
        dy = point.y - center_y;
        sum += std::pow(std::sqrt(dx * dx + dy * dy) - radius, 2);
    }
    return std::sqrt(sum / pointcloud.size());
}

bool CircleFit::pointcloudIsValid(std::vector<Point>& pointcloud)
{
    return pointcloud.size() >= 3;
}

Circle CircleFit::hyperFit(std::vector<Point>& pointcloud, int iter_max)
{
    unsigned long n = pointcloud.size();
    Point mean;
    real Xi, Yi, Zi;
    real Mxx, Mxy, Mxz, Myy, Myz, Mzz, Mz, Cov_xy, Var_z;
    real A0, A1, A2, A22;
    real Dy, x_new, x, y_new, y;
    real det;
    Point center;
    real radius;

    mean = calcMean(pointcloud);

    unsigned int mod = 0;
    // compute moments
    Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0;
    for (auto& point : pointcloud)
    {
        Xi = point.x - mean.x;
        Yi = (point.y + mod % 9 * 0.001) - mean.y;
        Zi = Xi * Xi + Yi * Yi;

        Mxy += Xi * Yi;
        Mxx += Xi * Xi;
        Myy += Yi * Yi;
        Mxz += Xi * Zi;
        Myz += Yi * Zi;
        Mzz += Zi * Zi;

        mod++;
    }
    Mxx /= n;
    Myy /= n;
    Mxy /= n;
    Mxz /= n;
    Myz /= n;
    Mzz /= n;

    // computing the coefficients of characteristic polynomial
    Mz = Mxx + Myy;
    Cov_xy = Mxx * Myy - Mxy * Mxy;
    Var_z = Mzz - Mz * Mz;

    A2 = 4.0 * Cov_xy - 3.0 * Mz * Mz - Mzz;
    A1 = Var_z * Mz + 4.0 * Cov_xy * Mz - Mxz * Mxz - Myz * Myz;
    A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - Var_z * Cov_xy;
    A22 = A2 + A2;

    // finding the root of the characteristic polynomial
    y = A0;
    x = 0.0;
    for (size_t i = 0; i < iter_max; i++)
    {
        Dy = A1 + x * (A22 + 16.0 * x * x);
        x_new = x - y / Dy;
        if (x_new == x || !std::isfinite(x_new))
            break;
        y_new = A0 + x_new * (A1 + x_new * (A2 + 4.0 * x_new * x_new));
        if (std::abs(y_new) >= std::abs(y))
            break;
        x = x_new;
        y = y_new;
    }
    det = x * x - x * Mz + Cov_xy;
    center.x = (Mxz * (Myy - x) - Myz * Mxy) / det / 2.0;
    center.y = (Myz * (Mxx - x) - Mxz * Mxy) / det / 2.0;

    radius = std::sqrt(std::abs(center.x * center.x + center.y * center.y + Mz));
    center.x = center.x + mean.x;
    center.y = center.y + mean.y;
    // sigma = sigma(data, circle.x_center, circle.y_center, circle.radius);

    return Circle(center, radius);
}