#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "icp.hpp"

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: ./main <num_sampling_points>" << std::endl;
        return EXIT_FAILURE;
    }
    const int num_sample = atoi(argv[1]);
    const double radius = 10.0;

    // generate sample data
    Eigen::MatrixXd data1 = Eigen::MatrixXd::Zero(num_sample, 2);
    generate_point_cloud(num_sample, radius, data1);

    // shift and rotate at random
    Eigen::MatrixXd data2 = Eigen::MatrixXd::Zero(num_sample, 2);
    random_shift(data1, data2);

    // run icp
    const int iter = icp(data1, data2);

    // plot
    plot_data(data1, data2, iter);
}