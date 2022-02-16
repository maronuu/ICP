#pragma once

class Point2D {
private:
    double x;
    double y;
public:
    double get_x();
    double get_y();
    void set_x(const double x_);
    void set_y(const double y_);
    void set_xy(const double x_, const double y_);
    void shift(const double dx, const double dy);
    void rotate(const double theta);
};