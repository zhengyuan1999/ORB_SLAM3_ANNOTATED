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

#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <sophus/se3.hpp>

#include <Eigen/Geometry>

#include "Converter.h"
#include "GeometricTools.h"

namespace ORB_SLAM3
{
/**
 * @brief 此类为抽象类，相机模型都应该继承此类，并实现本类提供的纯虚函数
*/
class GeometricCamera
{
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnType;
        ar & mvParameters;
    }

public:
    // 显示定义默认构造函数
    GeometricCamera() {}

    GeometricCamera(const std::vector<float> &_vParameters) : mvParameters(_vParameters) {}

    // 显示定义默认析构函数
    ~GeometricCamera() {}

    // 一堆投影成员函数
    virtual cv::Point2f project(const cv::Point3f &p3D) = 0;
    virtual Eigen::Vector2d project(const Eigen::Vector3d &v3D) = 0;
    virtual Eigen::Vector2f project(const Eigen::Vector3f &v3D) = 0;
    virtual Eigen::Vector2f projectMat(const cv::Point3f &p3D) = 0;

    // 这是干啥的？不确定性？
    virtual float uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) = 0;

    // 两个反向投影
    virtual Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) = 0;
    virtual cv::Point3f unproject(const cv::Point2f &p2D) = 0;

    // 因变量（投影到相机上的像素点）对自变量（路标点）的雅可比
    virtual Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d &v3D) = 0;

    /**
     * @brief 恢复运动结构
     * 
     * @param[in] vKeys1 Frame1 的关键点 vector
     * @param[in] vKeys2 Frame2 的关键点 vector
     * @param[in] vMatches12 其索引与 vKeys1 相对应，索引内容为与之匹配的关键点在 vKeys2 的索引
     * @param[out] T21 通过匹配的关键点恢复出来运动结构 T21
     * @param[out] vP3D 其索引与 vKeys1 相对应，索引内容为该关键点恢复出的路标点
     * @param[out] vbTriangulated 其索引与 vKeys1 相对应，索引内容为该关键点是否恢复出路标点
    */
    virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2, const std::vector<int> &vMatches12,
            Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated) = 0;

    virtual cv::Mat toK() = 0;
    virtual Eigen::Matrix3f toK_() = 0;

    virtual bool epipolarConstrain(GeometricCamera *otherCamera, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const Eigen::Matrix3f &R12, const Eigen::Vector3f &t12, const float sigmaLevel, const float unc) = 0;

    float getParameter(const int i) { return mvParameters[i]; }
    void setParameter(const float p, const size_t i) { mvParameters[i] = p; }

    size_t size() { return mvParameters.size(); }

    virtual bool matchAndtriangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, GeometricCamera *pOther,
            Sophus::SE3f &Tcw1, Sophus::SE3f &Tcw2, const float sigmaLevel1, const float sigmaLevel2, Eigen::Vector3f &x3Dtriangulated) = 0;

    unsigned int GetId() { return mnId; }

    unsigned int GetType() { return mnType; }

    const static unsigned int CAM_PINHOLE = 0;
    const static unsigned int CAM_FISHEYE = 1;

    static long unsigned int nNextId;

protected:
    std::vector<float> mvParameters; // 相机参数 {fx, fy, cx, cy}

    unsigned int mnId; // 相机 ID

    unsigned int mnType; // 相机类型（CAM_PINHOLE 或 CAM_FISHEYE）
};
}

#endif // CAMERAMODELS_GEOMETRICCAMERA_H
