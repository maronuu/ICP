#include "icp.hpp"

#include <iostream>
#include <random>
#include <vector>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

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

void random_shift(const Eigen::MatrixXd &src, Eigen::MatrixXd &dst)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> trans_dist(-100, 100);
    std::uniform_real_distribution<double> rot_dist(0, 2 * M_PI);
    const double dx = trans_dist(gen);
    const double dy = trans_dist(gen);
    const double theta = rot_dist(gen);
    printf("dx = %f, dy = %f, dtheta = %f\n", dx, dy, theta);
    const int num_sample = src.rows();
    for (int i = 0; i < num_sample; ++i)
    {
        // transition
        dst(i, 0) = src(i, 0) + dx;
        dst(i, 1) = src(i, 1) + dy;
        // rotation
        double tmp_x = dst(i, 0);
        double tmp_y = dst(i, 1);
        dst(i, 0) = tmp_x * cos(theta) - tmp_y * sin(theta);
        dst(i, 1) = tmp_x * sin(theta) + tmp_y * cos(theta);
    }
}

int icp(const Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2)
{
    assert(data1.rows() == data2.rows());
    const int num_point = data1.rows();
    Eigen::MatrixXd data1_var = data1; // deep copy

    constexpr double inf = 1e20;
    constexpr double eps = 1e-3;

    // correspondence[i] = j means (data1[i], data2[j]) is corresponding
    std::vector<int> correspondence(num_point, -1);
    double error_before = inf;
    double error_after = inf;

    int cnt = 0;
    while (cnt == 0 || std::abs(error_before - error_after) > eps)
    {
        error_before = error_after;
        error_after = icp_once(data1_var, data2, num_point, correspondence);
        if (cnt % 100 == 0)
        {
            printf("%d: error = %f\n", cnt, error_after);
        }
        cnt++;
    }

    printf("%d: error = %f\n", cnt, error_after);
    std::cout << "ICP Done" << std::endl;
    return cnt;
}

double icp_once(Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2, const int num_point, std::vector<int> &correspondence)
{
    constexpr double inf = 1e20;
    // nearest neighbor
    for (int i = 0; i < num_point; ++i)
    {
        double min_sq_dist = inf;
        int min_j = -1;
        for (int j = 0; j < num_point; ++j)
        {
            const auto dr = data1.row(i) - data2.row(j);
            const double sq_dist = dr.squaredNorm();
            if (sq_dist < min_sq_dist)
            {
                min_sq_dist = sq_dist;
                min_j = j;
            }
        }
        correspondence[i] = min_j;
    }

    // translation
    const auto translation = compute_translation(data1, data2, correspondence);
    for (int i = 0; i < num_point; ++i)
    {
        data1.row(i) += translation;
    }

    // rotation
    const auto rotation = compute_rotation(data1, data2, correspondence);
    for (int i = 0; i < num_point; ++i)
    {
        data1.row(i) *= rotation.transpose();
    }

    // calcurate error
    const double error = compute_error(data1, data2);
    return error;
}

Eigen::VectorXd compute_translation(const Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2, const std::vector<int> &correspondence)
{
    const auto c1 = compute_center_of_mass(data1);
    Eigen::MatrixXd corresp_data = Eigen::MatrixXd::Zero(data1.rows(), data1.cols());
    for (int i = 0; i < data1.rows(); ++i)
    {
        for (int j = 0; j < data1.cols(); ++j)
        {
            corresp_data(i, j) = data2(correspondence[i], j);
        }
        // corresp_data.row(i) = data2.row(correspondence[i]);
    }
    const auto c2 = compute_center_of_mass(corresp_data);
    return c2 - c1;
}

Eigen::MatrixXd compute_rotation(const Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2, const std::vector<int> &correspondence)
{
    // |R -  q p.T|
    // M: 2x2 matrix
    Eigen::MatrixXd M = data2.transpose() * data1;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd R = svd.matrixU() * svd.matrixV().transpose();
    return R;
}

Eigen::VectorXd compute_center_of_mass(const Eigen::MatrixXd &data)
{
    return data.colwise().mean();
}

double compute_error(const Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2)
{
    assert(data1.rows() == data2.rows());
    double ret = 0.0;
    for (int i = 0; i < data1.rows(); ++i)
    {
        ret += (data1.row(i) - data2.row(i)).squaredNorm();
    }
    return ret;
}
