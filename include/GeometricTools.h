/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GEOMETRIC_TOOLS_H
#define GEOMETRIC_TOOLS_H

#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Core>

namespace ORB_SLAM3
{

class KeyFrame;

class GeometricTools
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 计算 KeyFrame2 到 KeyFrame1 的基础矩阵 F12（Compute the Fundamental matrix between KF1 and KF2）
     * 
     * @param[in] pKF1 KeyFrame1
     * @param[in] pKF2 KeyFrame2
     * 
     * @return KeyFrame2 到 KeyFrame1 的基础矩阵 F12
    */
    static Eigen::Matrix3f ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

    /**
     * @brief 给定一对匹配关键点和这两帧的投影矩阵 [R|t] 三角化恢复路标点（Triangulate point with KF1 and KF2）
     * 
     * @param x_c1 来自 Frame1 的关键点 [x1, y1, 1]
     * @param x_c2 来自 Frame2 的关键点 [x2, y2, 1]
     * @param Tc1w Frame1 的投影矩阵
     * @param Tc2w Frame2 的投影矩阵
     * @param x3D 恢复出来的路标点
     * 
     * @return 三角化是否成功
    */
    static bool Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2, Eigen::Matrix<float, 3, 4> &Tc1w, Eigen::Matrix<float, 3, 4> &Tc2w, Eigen::Vector3f &x3D);


// 下面这两个模版成员静态函数没用
    // template <int rows, int cols>
    // static bool CheckMatrices(const cv::Mat &cvMat, const Eigen::Matrix<float, rows, cols> &eigMat)
    // {
    //     const float epsilon = 1e-3;
    //     // std::cout << cvMat.cols - cols << cvMat.rows - rows << std::endl;
    //     if (rows != cvMat.rows || cols != cvMat.cols)
    //     {
    //         std::cout << "wrong cvmat size\n";
    //         return false;
    //     }
    //     for (int i = 0; i < rows; i++)
    //         for (int j = 0; j < cols; j++)
    //             if ((cvMat.at<float>(i, j) > (eigMat(i, j) + epsilon)) ||
    //                 (cvMat.at<float>(i, j) < (eigMat(i, j) - epsilon)))
    //             {
    //                 std::cout << "cv mat:\n"
    //                             << cvMat << std::endl;
    //                 std::cout << "eig mat:\n"
    //                             << eigMat << std::endl;
    //                 return false;
    //             }
    //     return true;
    // }

    // template <typename T, int rows, int cols>
    // static bool CheckMatrices(const Eigen::Matrix<T, rows, cols> &eigMat1, const Eigen::Matrix<T, rows, cols> &eigMat2)
    // {
    //     const float epsilon = 1e-3;
    //     for (int i = 0; i < rows; i++)
    //         for (int j = 0; j < cols; j++)
    //             if ((eigMat1(i, j) > (eigMat2(i, j) + epsilon)) ||
    //                 (eigMat1(i, j) < (eigMat2(i, j) - epsilon)))
    //             {
    //                 std::cout << "eig mat 1:\n"
    //                             << eigMat1 << std::endl;
    //                 std::cout << "eig mat 2:\n"
    //                             << eigMat2 << std::endl;
    //                 return false;
    //             }
    //     return true;
    // }
};

} // namespace ORB_SLAM

#endif // GEOMETRIC_TOOLS_H
