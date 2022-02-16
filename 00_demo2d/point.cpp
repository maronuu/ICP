#include "point.hpp"
#include <math.h>


class Point2D
{
private:
    double x;
    double y;

public:
    double get_x() const
    {
        return x;
    }
    double get_y() const
    {
        return y;
    }
    void set_x(const double x_)
    {
        x = x_;
    }
    void set_y(const double y_)
    {
        y = y_;
    }
    void set_xy(const double x_, const double y_)
    {
        x = x_;
        y = y_;
    }
    void shift(const double dx, const double dy)
    {
        x += dx;
        y += dy;
    }
    void rotate(const double theta)
    {
        const double tmp_x = x;
        const double tmp_y = y;
        x = tmp_x * cos(theta) - tmp_y * sin(theta);
        y = tmp_x * sin(theta) + tmp_y * cos(theta);
    }
};