/*
 * Copyright (c) 2019 Nobuyuki Umetani
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

/**
 * @brief this demo just open window and it doesn't use DelFEM2 library
 */

#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>
#include <cstdlib>
#include <cstdio>

#if defined(_MSC_VER)
#pragma warning(disable : 4100)
#endif

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

int main()
{
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
    {
        exit(EXIT_FAILURE);
    }
    GLFWwindow *window = glfwCreateWindow(
        640, 480, "Simple example",
        nullptr, nullptr);
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
        glfwGetFramebufferSize(window, &width, &height);    // 現在の描画領域の大きさ
        const float ratio = (float)width / (float)height;
        glViewport(0, 0, width, height);    // 描画領域を指定
        glClear(GL_COLOR_BUFFER_BIT);   // RGB Color Bufferをクリア (背景色で塗りつぶし)
        // 描画
        glMatrixMode(GL_PROJECTION);    // projection matrix (3次元座標から正規デバイス座標系への変換)
        glLoadIdentity();   // 単位行列を設定
        glOrtho(-ratio, ratio, -1.f, 1.f, -1.f, +1.f);  // 描画範囲 
        glMatrixMode(GL_MODELVIEW); // カメラの外部パラメータ
        glLoadIdentity();
        glRotatef((float)glfwGetTime() * 50.f, 0.f, 0.f, 1.f);  // z軸周りに回転
        glPointSize(10);
        glBegin(GL_POINTS);  // 図形の定義begin
        glColor3f(1.f, 0.f, 0.f);   // RGB
        glVertex3f(-0.6f, -0.4f, 0.f);  // vertexの座標
        glColor3f(0.f, 1.f, 0.f);
        glVertex3f(0.6f, -0.4f, 0.f);
        glColor3f(0.f, 0.f, 1.f);
        glVertex3f(0.f, 0.6f, 0.f);
        glEnd();    // 図形の定義end
        glfwSwapBuffers(window);    // 2つあるbufferのswap -> viewの更新
        glfwPollEvents();   // callback
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}