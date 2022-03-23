#include "icp.hpp"

#include <iostream>
#include <random>
#include <vector>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include <GLFW/glfw3.h>
#include <cstdlib>
#include <cstdio>

static void error_callback([[maybe_unused]] int error, const char *description)
{
    fputs(description, stderr);
}

static void key_callback(GLFWwindow *window, int key, [[maybe_unused]] int scancode, int action, [[maybe_unused]] int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}

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
        // rotation
        // double tmp_x = dst(i, 0);
        // double tmp_y = dst(i, 1);
        // dst(i, 0) = tmp_x * cos(theta) - tmp_y * sin(theta);
        // dst(i, 1) = tmp_x * sin(theta) + tmp_y * cos(theta);
        // transition
        dst(i, 0) = src(i, 0) + dx;
        dst(i, 1) = src(i, 1) + dy;
    }
}

int icp(const Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2, Eigen::MatrixXd &res)
{
    assert(data1.rows() == data2.rows());
    const int num_point = data1.rows();
    assert(data1.cols() == data2.cols() && data2.cols() == res.cols());
    // deep copy
    for (int i = 0; i < num_point; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            res(i, j) = data1(i, j);
        }
    }
    // Eigen::MatrixXd data1_var = data1; // deep copy

    constexpr double inf = 1e20;
    constexpr double eps = 1e-2;

    // correspondence[i] = j means (data1[i], data2[j]) is corresponding
    std::vector<int> correspondence(num_point, -1);
    double error_before = inf;
    double error_after = inf;

    int cnt = 0;
    while (cnt == 0 || std::abs(error_before - error_after) > eps)
    {
        error_before = error_after;
        error_after = icp_once(res, data2, num_point, correspondence);
        if (cnt % 100 == 0)
        {
            printf("%d: error = %f\n", cnt, error_after);
        }
        cnt++;
    }

    printf("%d: error = %f\n", cnt, error_after);
    printf("ICP Done\n");

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

void visualize(const Eigen::MatrixXd &data1, const Eigen::MatrixXd &data2)
{
    const int num_data = data1.rows();
    assert(data1.rows() == data2.rows()); // same number of points
    assert(data1.cols() == data2.cols()); // same dimension
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
    {
        exit(EXIT_FAILURE);
    }
    GLFWwindow *window = glfwCreateWindow(
        640, 480, "ICP Process", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, key_callback);
    while (!glfwWindowShouldClose(window))
    {
        int width, height;
        glfwGetFramebufferSize(window, &width, &height); // 現在の描画領域の大きさ
        const float ratio = (float)width / (float)height;
        glViewport(0, 0, width, height); // 描画領域を指定
        glClear(GL_COLOR_BUFFER_BIT);    // RGB Color Bufferをクリア (背景色で塗りつぶし)
        // 描画
        glMatrixMode(GL_PROJECTION);                   // projection matrix (3次元座標から正規デバイス座標系への変換)
        glLoadIdentity();                              // 単位行列を設定
        glOrtho(-ratio, ratio, -1.f, 1.f, -1.f, +1.f); // 描画範囲
        glMatrixMode(GL_MODELVIEW);                    // カメラの外部パラメータ
        glLoadIdentity();
        // glPointSize(10);
        //
        double max_x1 = 0.0f;
        double max_y1 = 0.0f;
        double max_x2 = 0.0f;
        double max_y2 = 0.0f;
        for (int i = 0; i < num_data; ++i)
        {
            if (max_x1 < data1(i, 0))
            {
                max_x1 = data1(i, 0);
            }
            if (max_y1 < data1(i, 1))
            {
                max_y1 = data1(i, 1);
            }
            if (max_x2 < data2(i, 0))
            {
                max_x2 = data2(i, 0);
            }
            if (max_y2 < data2(i, 1))
            {
                max_y2 = data2(i, 1);
            }
        }
        const double max_x = (max_x1 > max_x2) ? max_x1 : max_x2;
        const double max_y = (max_y1 > max_y2) ? max_y1 : max_y2;
        glBegin(GL_POINTS); // 図形の定義begin
        for (int i = 0; i < num_data; ++i)
        {
            glColor3f(1.f, 0.f, 0.f);
            glVertex3f(data1(i, 0) / max_x, data1(i, 1) / max_y, 0.f);
        }
        for (int i = 0; i < num_data; ++i)
        {
            glColor3f(0.f, 1.f, 0.f);
            glVertex3f(data2(i, 0) / max_x, data2(i, 1) / max_y, 0.f);
        }
        glEnd();                 // 図形の定義end
        glfwSwapBuffers(window); // 2つあるbufferのswap -> viewの更新
        glfwPollEvents();        // callback
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}