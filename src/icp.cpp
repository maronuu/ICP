#include "icp.hpp"

#include <iostream>
#include <random>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>

// generate sample point cloud (random sampling from circle)
void generate_point_cloud(const int num_sample, const double radius, Eigen::MatrixXd &data)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0, 2 * M_PI);

    double theta;
    // Eigen::MatrixXd data = Eigen::MatrixXd::Zero(num_sample, 2);
    for (int i = 0; i < num_sample; ++i)
    {
        theta = dis(gen);
        data(i, 0) = radius * cos(theta);
        data(i, 1) = radius * sin(theta);
    }
}

void random_shift(const Eigen::MatrixXd &src, Eigen::MatrixXd &dst) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> trans_dist(-1000, 1000);
    std::uniform_real_distribution<double> rot_dist(0, 2*M_PI);
    const double dx = trans_dist(gen);
    const double dy = trans_dist(gen);
    const double theta = rot_dist(gen);
    const int num_sample = src.rows();
    for (int i = 0; i < num_sample; ++i) {
        dst(i, 0) = src(i, 0) + dx;
        dst(i, 1) = src(i, 1) + dy;
        const double tmp_x = dst(i, 0);
        const double tmp_y = dst(i, 1);
        dst(i, 0) = tmp_x * cos(theta) - tmp_y * sin(theta);
        dst(i, 1) = tmp_x * sin(theta) + tmp_y * cos(theta);
    }
}

void icp(const Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2, std::vector<int> &correspondence)
{
    const int num_sample = data1.rows();
    assert(data1.rows() == data2.rows());

    // std::vector<int> correspondence(num_sample, -1);
    double dx, dy, sq_dist;
    std::vector<bool> is_matched(num_sample, false);
    for (int i = 0; i < num_sample; ++i)
    {
        double min_sq_dist = 1e9;
        int min_j = -1;
        for (int j = 0; j < num_sample; ++j)
        {
            if (is_matched[j]) continue;
            dx = data1(i, 0) - data2(j, 0);
            dy = data1(i, 1) - data2(j, 1);
            sq_dist = dx * dx + dy * dy;
            if (sq_dist < min_sq_dist)
            {
                min_sq_dist = sq_dist;
                min_j = j;
            }
        }
        correspondence[i] = min_j;
        is_matched[min_j] = true;
    }
}