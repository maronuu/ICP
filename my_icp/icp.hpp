#pragma once

#include <eigen3/Eigen/Dense>

void generate_point_cloud(const int num_sample, const double radius, Eigen::MatrixXd &data);

void random_shift(const Eigen::MatrixXd &src, Eigen::MatrixXd &dst);

void icp(const Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2);

void icp_once(Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2, std::vector<int> &correspondence);

Eigen::VectorXd compute_translation(const Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2, const std::vector<int> &correspondence);

Eigen::MatrixXd compute_rotation(const Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2, const std::vector<int> &correspondence);

Eigen::VectorXd compute_center_of_mass(const Eigen::MatrixXd &data);