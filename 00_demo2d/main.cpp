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
    const double radius = 1.0;
    printf("num_point = %d\n", num_sample);

    // generate sample data
    Eigen::MatrixXd src_data = Eigen::MatrixXd::Zero(num_sample, 2);
    generate_point_cloud(num_sample, radius, src_data);
    // shift and rotate at random
    Eigen::MatrixXd dst_data = Eigen::MatrixXd::Zero(num_sample, 2);
    random_shift(src_data, dst_data);

    visualize(src_data, dst_data);
    // run icp
    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(num_sample, 2);
    const int iter = icp(src_data, dst_data, res);
    std::cout << "Iteration = " << iter << std::endl;
    // visualize (after)
    visualize(res, dst_data);
}