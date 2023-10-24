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

#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "sophus/sim3.hpp"

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace ORB_SLAM3
{

class ORBmatcher
{
public:
    ORBmatcher(float nnratio = 0.6, bool checkOri = true);

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(Frame &F, const std::vector<MapPoint *> &vpMapPoints, const float th = 3, const bool bFarPoints = false, const float thFarPoints = 50.0f);

    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    int SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const std::set<MapPoint *> &sAlreadyFound, const float th, const int ORBdist);

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
    int SearchByProjection(KeyFrame *pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint *> &vpPoints, std::vector<MapPoint *> &vpMatched, int th, float ratioHamming = 1.0);

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in Place Recognition (Loop Closing and Merging)
    int SearchByProjection(KeyFrame *pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint *> &vpPoints, const std::vector<KeyFrame *> &vpPointsKFs, std::vector<MapPoint *> &vpMatched, std::vector<KeyFrame *> &vpMatchedKF, int th, float ratioHamming = 1.0);

    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    /**
     * @brief Tracking::TrackReferenceKeyFrame() 和 Tracking::Relocalization() 中 F 对 pKF 特征匹配以得到 F 的特征点对应的路标点
     * 
     * @param[in] pKF 关键帧
     * @param[in] F 帧
     * @param[out] vpMapPointMatches F 的特征点对应的路标点，其长度等于 F.N
     * 
     * @return 成功匹配的数量（得到 F 的特征点对应的路标点数量）
    */
    int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint *> &vpMapPointMatches);

    /**
     * @brief LoopClosing::DetectCommonRegionsFromBoW() 中 pKF2 对 pKF1 特征匹配以得到 pKF2 的特征点对应的路标点
     * 
     * @param[in] pKF1 关键帧 1
     * @param[in] pKF2 关键帧 2
     * @param[out] vpMatches12 pKF2 的特征点对应的路标点，其长度等于 F.N
     * 
     * @return 成功匹配的数量（得到 pKF2 的特征点对应的路标点数量）
    */
    int SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches12);

    /**
     * @brief 单目初始化中当前帧对初始帧特征匹配（Matching for the Map Initialization (only used in the monocular case)）
     * 
     * @note 本成员函数只使用原始图像，不使用图像金字塔
     * 
     * @param[in] F1 初始帧 Tracking::mInitialFrame
     * @param[in] F2 当前帧 Tracking::mCurrentFrame
     * @param[in out] vbPrevMatched 上一帧对初始帧的特征匹配结果
     * @param[in] vnMatches12 F2 对 F1 的特征匹配结果
     * @param[in] windowSize 搜索窗口的大小
     * 
     * @return 成功匹配的数量
    */
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize = 10);

    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2,
            std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse = false);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    // int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
    int SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches12, const Sophus::Sim3f &S12, const float th);

    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th = 3.0, const bool bRight = false);

    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(KeyFrame *pKF, Sophus::Sim3f &Scw, const std::vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

    // Computes the Hamming distance between two ORB descriptors
    // 计算两个 BRIEF 描述子汉明距离（与 Thirdparty/DBoW2/DBoW2/FORB.cpp 中的 FORB::distance 完全一致）
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;

protected:
    // return (viewCos>0.998 ? 2.5 : 4.0);
    float RadiusByViewingCos(const float &viewCos);

    /**
     * @brief 寻找方向梯度直方图中前三“高”的索引
     * 
     * @param[in] histo 方向梯度直方图
     * @param[in] L 方向梯度直方图索引长度
     * @param[out] ind1 第一“高”
     * @param[out] ind2 第二“高”
     * @param[out] ind3 第三“高”
    */
    void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio; // 最优描述子距离和次优描述子距离应满足的比例系数

    bool mbCheckOrientation; // 是否检查特征点方向
};

} // namespace ORB_SLAM

#endif // ORBMATCHER_H
