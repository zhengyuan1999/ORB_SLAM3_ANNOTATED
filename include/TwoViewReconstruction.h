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

#ifndef TwoViewReconstruction_H
#define TwoViewReconstruction_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <unordered_set>
#include <sophus/se3.hpp>

namespace ORB_SLAM3
{

class TwoViewReconstruction
{
    // 第一个 int 来自 Frame 1，保存着 mvKeys1 的索引
    // 第二个 int 来自 Frame 2，保存着 mvKeys2 的索引
    typedef std::pair<int, int> Match;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Fix the reference frame
    /**
     * @brief 初始化成员对象
     * 
     * @param[in] k 相机内参矩阵
     * @param[in] sigma 标准差
     * @param[in] iterations RANSAC 算法最大迭代次数
    */
    TwoViewReconstruction(const Eigen::Matrix3f &k, float sigma = 1.0, int iterations = 200);

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    /**
     * @brief 同时计算基础矩阵和单应矩阵，并根据得分结果来选择使用哪个矩阵恢复运动结构
     * 
     * @param[in] vKeys1 Frame 1 的关键点 vector
     * @param[in] vKeys2 Frame 2 的关键点 vector
     * @param[in] vMatches12 其索引与 vKeys1 相对应，索引内容为与之匹配的关键点在 vKeys2 的索引
     * @param[out] T21 通过匹配的关键点恢复出来运动结构 T21
     * @param[out] vP3D 其索引与 vKeys1 相对应，索引内容为该关键点恢复出的路标点
     * @param[out] vbTriangulated 其索引与 vKeys1 相对应，索引内容为该关键点是否恢复出路标点
    */
    bool Reconstruct(const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2, const std::vector<int> &vMatches12,
            Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);

private:

    /**
     * @brief 解算出 RANSAC 集合所有单应矩阵 H21，并通过函数参数返回得分最高者
     * 
     * @param[out] vbMatchesInliers 其索引与 mvMatches12 相对应，索引内容为该匹配是否为最优单应矩阵的内点
     * @param[out] score 最优的得分
     * @param[out] H21 解算出来得分最优的单应矩阵 H21
    */
    void FindHomography(std::vector<bool> &vbMatchesInliers, float &score, Eigen::Matrix3f &H21);
    
    /**
     * @brief 解算出 RANSAC 集合所有基础矩阵 F21，并通过函数参数返回得分最高者
     * 
     * @param[out] vbMatchesInliers 其索引与 mvMatches12 相对应，索引内容为该匹配是否为最优基础矩阵的内点
     * @param[out] score 最优的得分
     * @param[out] H21 解算出来得分最优的基础矩阵 H
    */
    void FindFundamental(std::vector<bool> &vbInliers, float &score, Eigen::Matrix3f &F21);

    /**
     * @brief 给定 8 点对计算单应矩阵
     * 
     * @param[in] vP1 来自 Frame 1 的 8 个关键点
     * @param[in] vP2 来自 Frame 2 的 8 个关键点，与 vP1 的关键点一一匹配
     * 
     * @return 解算出来的单应矩阵 H21
    */
    Eigen::Matrix3f ComputeH21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);
    
    /**
     * @brief 给定 8 点对计算基础矩阵
     * 
     * @param[in] vP1 来自 Frame 1 的 8 个关键点
     * @param[in] vP2 来自 Frame 2 的 8 个关键点，与 vP1 的关键点一一匹配
     * 
     * @return 解算出来的基础矩阵 F21
    */
    Eigen::Matrix3f ComputeF21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);

    /**
     * @brief 通过重投影误差计算 H21 的分数
     * 
     * @param[in] H21 单应矩阵 H21
     * @param[in] H12 单应矩阵 H12
     * @param[out] vbMatchesInliers 其索引与 mvMatches12 相对应，索引内容为该匹配是否为 H21 的内点
     * @param[in] sigma 标准差
     * 
     * @return 得分
    */
    float CheckHomography(const Eigen::Matrix3f &H21, const Eigen::Matrix3f &H12, std::vector<bool> &vbMatchesInliers, float sigma);

    /**
     * @brief 通过重投影误差计算 F21 的分数
     * 
     * @param[in] F21 单应矩阵 F21
     * @param[out] vbMatchesInliers 其索引与 mvMatches12 相对应，索引内容为该匹配是否为 F21 的内点
     * @param[in] sigma 标准差
     * 
     * @return 得分
    */
    float CheckFundamental(const Eigen::Matrix3f &F21, std::vector<bool> &vbMatchesInliers, float sigma);

    /**
     * @brief 从给定的基础矩阵中恢复运动结构，并三角化恢复路标点
     * 
     * @param[in] vbMatchesInliers 其索引与 mvMatches12 相对应，索引内容为该匹配是否为 F21 的内点
     * @param[in] F21 Frame 1 到 Frame 2 的基础矩阵
     * @param[in] K 相机内参矩阵
     * @param[out] T21 通过基础矩阵恢复出来的变换矩阵
     * @param[out] vP3D 其索引与 mvKeys1 相对应，索引内容为该关键点恢复出的路标点
     * @param[out] vbTriangulated 其索引与 mvKeys1 相对应，索引内容为该关键点是否恢复出路标点
     * @param[in] minParallax 认为三角化有效的最小视差角
     * @param[in] minTriangulated 三角化恢复的最少路标点数量（默认 50 与 0.9 倍内点的最大值）
     * 
     * @return 是否通过恢复的运动结构三角化恢复出足够多且准确的路标点
    */
    bool ReconstructF(std::vector<bool> &vbMatchesInliers, Eigen::Matrix3f &F21, Eigen::Matrix3f &K,
            Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    bool ReconstructH(std::vector<bool> &vbMatchesInliers, Eigen::Matrix3f &H21, Eigen::Matrix3f &K,
            Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    /**
     * @brief 对给定的关键点 vector 进行归一化，这是计算基础矩阵和单应矩阵的前置步骤
     * 
     * @param[in] vKeys 原始的关键点
     * @param[in] vNormalizedPoints 归一化的关键点
     * @param[in] T 归一化矩阵 vNormalizedPoints[i] = T * vKeys[i]
    */
    void Normalize(const std::vector<cv::KeyPoint> &vKeys, std::vector<cv::Point2f> &vNormalizedPoints, Eigen::Matrix3f &T);

    /**
     * @brief 
     * 
     * @param[in] R 可能的旋转矩阵
     * @param[in] t 可能的位置向量
     * @param[in] vKeys1 Frame 1 的关键点 vector
     * @param[in] vKeys2 Frame 2 的关键点 vector
     * @param[in] vMatches12 索引与 vKeys1 相对应，索引内容为与之匹配的关键点在 vKeys2 的索引
     * @param[in] vbMatchesInliers 其索引与 mvMatches12 相对应，索引内容为该匹配是否为 F21 的内点
     * @param[in] K 相机内参矩阵
     * @param[out] vP3D 其索引与 mvKeys1 相对应，索引内容为该关键点恢复出的路标点
     * @param[in] th2 
     * @param[out] vbGood 其索引与 mvKeys1 相对应，索引内容为该关键点是否恢复出路标点
     * 
     * @return 三角化恢复的路标点数量
    */
    int CheckRT(const Eigen::Matrix3f &R, const Eigen::Vector3f &t, const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2,
            const std::vector<Match> &vMatches12, std::vector<bool> &vbMatchesInliers,
            const Eigen::Matrix3f &K, std::vector<cv::Point3f> &vP3D, float th2, std::vector<bool> &vbGood, float &parallax);

    /**
     * @brief 由本质矩阵 E 恢复可能的运动结构
     * 
     * @param[in] E 本质矩阵
     * @param[out] R1 可能的旋转矩阵
     * @param[out] R2 可能的旋转矩阵
     * @param[out] t 可能的位置向量（另一个可能的位置向量为 -t）
    */
    void DecomposeE(const Eigen::Matrix3f &E, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2, Eigen::Vector3f &t);


// mvKeys1、mvKeys2、mvMatches12 和 mvbMatched1 只在 Reconstruct 中被赋值
    // 来自参考帧（Frame 1）的关键点 vector（Keypoints from Reference Frame (Frame 1)）
    std::vector<cv::KeyPoint> mvKeys1;

    // 来自当前帧（Frame 2）的关键点 vector（Keypoints from Current Frame (Frame 2)）
    std::vector<cv::KeyPoint> mvKeys2;

    // Current Matches from Reference to Current
    std::vector<Match> mvMatches12; // 存储着 Frame 1 和 Frame 2 匹配的关键点对
    std::vector<bool> mvbMatched1;  // 存储着 Frame 1 的关键点是否匹配到了 Frame 2 的关键点


// mk、mSigma、mSigma2 和 mMaxIterations 只在构造函数中被赋值
    Eigen::Matrix3f mK;    // 相机内参矩阵（Calibration）
    float mSigma, mSigma2; // 标准差和方差（Standard Deviation and Variance）
    int mMaxIterations;    // RANSAC 算法最大迭代次数（Ransac max iterations）


// mvSets 只在 Reconstruct 中被赋值 
    // 第一层 vector 长度为 mMaxIterations，保存所有 RANSAC 集合元素
    // 第二层 vector 长度为 8，每个集合元素为 8 个点对，索引内容为 mvMatches12 的索引
    std::vector<std::vector<size_t>> mvSets; // Ransac sets
};

} // namespace ORB_SLAM

#endif // TwoViewReconstruction_H
