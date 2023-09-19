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

#ifndef ORB_SLAM3_SETTINGS_H
#define ORB_SLAM3_SETTINGS_H

// Flag to activate the measurement of time in each process (track,localmap, place recognition).
// #define REGISTER_TIMES

#include "CameraModels/GeometricCamera.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

namespace ORB_SLAM3
{

class System;

// -TODO: change to double instead of float

class Settings
{
public:
    // 相机模型（Enum for the different camera types implemented）
    enum CameraType
    {
        PinHole       = 0, // 针孔相机模型
        Rectified     = 1, // 已去畸变针孔相机模型
        KannalaBrandt = 2  // 鱼眼相机模型
    };

    // 不允许无参构造（Delete default constructor）
    Settings() = delete;

    // Constructor from file
    Settings(const std::string &configFile, const int &sensor);

    // Ostream operator overloading to dump settings to the terminal
    friend std::ostream &operator<<(std::ostream &output, const Settings &s);

    // Getter methods
    CameraType cameraType() { return cameraType_; }
    GeometricCamera *camera1() { return calibration1_; }
    GeometricCamera *camera2() { return calibration2_; }
    cv::Mat camera1DistortionCoef() { return cv::Mat(vPinHoleDistorsion1_.size(), 1, CV_32F, vPinHoleDistorsion1_.data()); }
    cv::Mat camera2DistortionCoef() { return cv::Mat(vPinHoleDistorsion2_.size(), 1, CV_32F, vPinHoleDistorsion1_.data()); }
    Sophus::SE3f Tlr() { return Tlr_; }
    float bf() { return bf_; }
    float b() { return b_; }
    float thDepth() { return thDepth_; }
    bool needToUndistort() { return bNeedToUndistort_; }
    cv::Size newImSize() { return newImSize_; }
    float fps() { return fps_; }
    bool rgb() { return bRGB_; }
    bool needToResize() { return bNeedToResize1_; }
    bool needToRectify() { return bNeedToRectify_; }
    float noiseGyro() { return noiseGyro_; }
    float noiseAcc() { return noiseAcc_; }
    float gyroWalk() { return gyroWalk_; }
    float accWalk() { return accWalk_; }
    float imuFrequency() { return imuFrequency_; }
    Sophus::SE3f Tbc() { return Tbc_; }
    bool insertKFsWhenLost() { return insertKFsWhenLost_; }
    float depthMapFactor() { return depthMapFactor_; }
    int nFeatures() { return nFeatures_; }
    int nLevels() { return nLevels_; }
    float initThFAST() { return initThFAST_; }
    float minThFAST() { return minThFAST_; }
    float scaleFactor() { return scaleFactor_; }
    float keyFrameSize() { return keyFrameSize_; }
    float keyFrameLineWidth() { return keyFrameLineWidth_; }
    float graphLineWidth() { return graphLineWidth_; }
    float pointSize() { return pointSize_; }
    float cameraSize() { return cameraSize_; }
    float cameraLineWidth() { return cameraLineWidth_; }
    float viewPointX() { return viewPointX_; }
    float viewPointY() { return viewPointY_; }
    float viewPointZ() { return viewPointZ_; }
    float viewPointF() { return viewPointF_; }
    float imageViewerScale() { return imageViewerScale_; }
    std::string atlasLoadFile() { return sLoadFrom_; }
    std::string atlasSaveFile() { return sSaveto_; }
    float thFarPoints() { return thFarPoints_; }
    cv::Mat M1l() { return M1l_; }
    cv::Mat M2l() { return M2l_; }
    cv::Mat M1r() { return M1r_; }
    cv::Mat M2r() { return M2r_; }

private:
    template <typename T>
    T readParameter(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required = true)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return T();
            }
        }
        else
        {
            found = true;
            return (T)node;
        }
    }

    void readCamera1(cv::FileStorage &fSettings);
    void readCamera2(cv::FileStorage &fSettings);
    void readImageInfo(cv::FileStorage &fSettings);
    void readIMU(cv::FileStorage &fSettings);
    void readRGBD(cv::FileStorage &fSettings);
    void readORB(cv::FileStorage &fSettings);
    void readViewer(cv::FileStorage &fSettings);
    void readLoadAndSave(cv::FileStorage &fSettings);
    void readOtherParameters(cv::FileStorage &fSettings);

    void precomputeRectificationMaps();

    int sensor_;
    CameraType cameraType_; // Camera type

    // Visual stuff
    GeometricCamera *calibration1_, *calibration2_; // Camera calibration
    GeometricCamera *originalCalib1_, *originalCalib2_;
    std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_;

    cv::Size originalImSize_; // 原始图像尺寸
    cv::Size  newImSize_;     // 修改后的图像尺寸
    float fps_;               // 相机频率（帧 / s）
    bool bRGB_;               // 彩色图像通道顺序是否为 RGB

    bool bNeedToUndistort_; // 是否需要去畸变（相机为单目相机且 YAML 文件中没有畸变参数时不需要去畸变）
    bool bNeedToRectify_;   // 是否需要双目校正
    bool bNeedToResize1_;   // 是否需要改变左图像尺寸
    bool bNeedToResize2_;   // 是否需要改变左图像尺寸

    Sophus::SE3f Tlr_; // 右相机到左相机的变换矩阵
    float thDepth_;    // 深度阈值
    float bf_;         // b * fx
    float b_;          // 基线

    // Rectification stuff
    cv::Mat M1l_, M2l_;
    cv::Mat M1r_, M2r_;

    // Inertial stuff
    float noiseGyro_, noiseAcc_; // IMU 陀螺仪偏置和加速度计偏置
    float gyroWalk_, accWalk_;   // IMU 陀螺仪随机游走和加速度计随机游走
    float imuFrequency_;         // IMU 频率
    Sophus::SE3f Tbc_;           // （左）相机到 IMU 的变换矩阵
    bool insertKFsWhenLost_;

    // RGBD stuff
    float depthMapFactor_; // 深度系数（米 = 深度 / depthMapFactor_）

    // ORB stuff
    int nFeatures_;     // 关键点数量
    float scaleFactor_; // 图像金字塔缩放系数
    int nLevels_;       // 图像金字塔层数
    int initThFAST_;    // FAST 关键点提取阈值（高）
    int minThFAST_;     // FAST 关键点提取阈值（低）

    // Viewer stuff
    float keyFrameSize_;
    float keyFrameLineWidth_;
    float graphLineWidth_;
    float pointSize_;
    float cameraSize_;
    float cameraLineWidth_;
    float viewPointX_, viewPointY_, viewPointZ_, viewPointF_;
    float imageViewerScale_;

    // Save & load maps
    std::string sLoadFrom_, sSaveto_;

    // Other stuff
    float thFarPoints_;
};
};

#endif // ORB_SLAM3_SETTINGS_H
