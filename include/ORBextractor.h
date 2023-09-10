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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM3
{

class ExtractorNode
{
public:
    ExtractorNode() : bNoMore(false) {}

    /**
     * @brief 将 1 个节点分裂为 4 个节点
     * 
     * @param[in] n1 分裂出来的节点 1（左上）
     * @param[in] n2 分裂出来的节点 2（右上）
     * @param[in] n3 分裂出来的节点 3（左下）
     * @param[in] n4 分裂出来的节点 4（右下）
    */
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys; // 属于本节点的关键点 vector

    cv::Point2i UL, UR, BL, BR; // 本节点的四个边界坐标

    std::list<ExtractorNode>::iterator lit; // 保存自己在链表中的迭代器

    bool bNoMore; // 如果本节点中只有一个关键点，说明本节点不可再分裂，将其置为 true
};

class ORBextractor
{
public:
    enum
    {
        HARRIS_SCORE = 0,
        FAST_SCORE = 1
    };

    /**
     * @brief
     * 
     * @param[in] nfeatures 关键点提取数量（默认 1200）
     * @param[in] scaleFactor 图像金字塔缩放尺度（默认 1.2）
     * @param[in] nlevels 图像金字塔缩放等级（默认 8）
     * @param[in] iniThFAST FAST 关键点初始阈值（默认 20）
     * @param[in] minThFAST 当使用初始阈值没有提取到关键点时用本最小阈值（默认 7）
    */
    ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST);

    /**
     * @brief 没用到的析构函数，可以不写
    */
    // ~ORBextractor() {}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    /**
     * @brief
     * 
     * @param[in] _image
     * @param[in] _mask 未被实现（Mask is ignored in the current implementation.）
     * @param[out] _keypoints 提取到的关键点 vector
     * @param[out] _descriptors 与 _keypoints 对应的描述子矩阵
     * @param[out] vLappingArea
     * 
     * @return 
    */
    int operator()(cv::InputArray _image, cv::InputArray _mask,
            std::vector<cv::KeyPoint> &_keypoints, cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

    int inline GetLevels() { return nlevels; }

    float inline GetScaleFactor() { return scaleFactor; }

    std::vector<float> inline GetScaleFactors() { return mvScaleFactor; }

    std::vector<float> inline GetInverseScaleFactors() { return mvInvScaleFactor; }

    std::vector<float> inline GetScaleSigmaSquares() { return mvLevelSigma2; }

    std::vector<float> inline GetInverseScaleSigmaSquares() { return mvInvLevelSigma2; }

    std::vector<cv::Mat> mvImagePyramid; // 图像金字塔

protected:
    /**
     * @brief 计算图像金字塔，并将计算结果赋值到 mvImagePyramid
     * 
     * @bug 该方法与预期不一致！实际上 mvImagePyramid 单纯为缩放后的图像，没有真正对边界进行填充
     * 
     * @param[in] image 图像
    */
    void ComputePyramid(cv::Mat image);

    /**
     * @brief 使用八叉树法（四叉树法）提取图像金字塔的关键点
     * 
     * @param[out] allKeypoints 第一层 vector 长度为 8，对应 8 层图像金字塔，第二层 vector 保存当前层提取到的关键点
    */
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> &allKeypoints);

    /**
     * 
    */
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
            const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    // void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint>> &allKeypoints); // 没用


// 以下所有变量全部在构造方法中被计算和赋值，并且不会再发生改变
    std::vector<cv::Point> pattern; // 

    int nfeatures;      // 关键点提取数量（默认 1200）
    double scaleFactor; // 图像金字塔缩放尺度（默认 1.2）
    int nlevels;        // 图像金字塔缩放等级（默认 8）
    int iniThFAST;      // FAST 关键点初始阈值（默认 20）
    int minThFAST;      // 当使用初始阈值没有提取到关键点时用本最小阈值（默认 7）

    // 每层图像金字塔需要提取关键点的数量
    std::vector<int> mnFeaturesPerLevel; // [261, 217, 181, 151, 126, 105, 87, 72]

    // 使用灰度质心法计算关键点方向时，固定 v 坐标，u 坐标可到达的最大值
    std::vector<int> umax; // [15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10, 9, 8, 6, 3]

    std::vector<float> mvScaleFactor;    // [1, 1.20, 1.44, 1.73, 2.07, 2.49, 2.99, 3.58]
    std::vector<float> mvInvScaleFactor; // [1, 0.83, 0.69, 0.58, 0.48, 0.40, 0.33, 0.28]
    std::vector<float> mvLevelSigma2;    // [1, 1.44, 2.07, 2.99, 4.30, 6.19, 8.92, 12.84]
    std::vector<float> mvInvLevelSigma2; // [1, 0.69, 0.48, 0.33, 0.23, 0.16, 0.11, 0.08]
};

} // namespace ORB_SLAM

#endif
