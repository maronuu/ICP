#include <iostream>
#include <vector>

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
    std::vector<int> correspondence(num_sample, -1);
    icp(data1, data2, correspondence);

    // plot
    for (int i = 0; i < num_sample; ++i)
    {
        std::cout << i << ": " << i << " -> " << correspondence[i] << std::endl;
    }
}