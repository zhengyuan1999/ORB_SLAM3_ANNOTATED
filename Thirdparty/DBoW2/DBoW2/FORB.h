/**
 * File: FORB.h
 * Date: June 2012
 * Author: Dorian Galvez-Lopez
 * Description: functions for ORB descriptors
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_F_ORB__
#define __D_T_F_ORB__

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

#include "FClass.h"

namespace DBoW2
{

// ORB 描述子（BRIEF描述子）处理类（Functions to manipulate ORB descriptors）
class FORB : protected FClass
{
public:
    // 描述子存储类型（Descriptor type）
    typedef cv::Mat TDescriptor; // CV_8U .size() = [32 x 1]（1 行 32 列）

    /// Pointer to a single descriptor
    typedef const TDescriptor *pDescriptor;

    // 描述子长度，单位为字节，BRIEF 描述子 256 b，也就是 32 B（Descriptor length (in bytes)）
    static const int L;

    /**
     * @brief 计算给定描述子集合的均值描述子（Calculates the mean value of a set of descriptors）
     * 
     * @param[in] descriptors 描述子集合
     * @param[out] mean 均值描述子（mean descriptor）
     */
    static void meanValue(const std::vector<pDescriptor> &descriptors, TDescriptor &mean);

    /**
     * @brief 计算一对描述子之间的汉明距离（Calculates the distance between two descriptors）
     * 
     * @return 一对描述子之间的汉明距离（distance）
     */
    static int distance(const TDescriptor &a, const TDescriptor &b);

    /**
     * @brief 返回字符串版本的描述子（Returns a string version of the descriptor）
     * 
     * @param[in] a 描述子
     * 
     * @return 描述子的字符串版本（string version）
     */
    static std::string toString(const TDescriptor &a);

    /** 
     * @brief 从字符串创建描述子（Returns a descriptor from a string）
     * 
     * @param[out] a 创建的描述子（descriptor）
     * @param[in] s 字符串（string version）
     */
    static void fromString(TDescriptor &a, const std::string &s);

    /**
     * Returns a mat with the descriptors in float format
     * @param descriptors
     * @param mat (out) NxL 32F matrix
     */
    static void toMat32F(const std::vector<TDescriptor> &descriptors, cv::Mat &mat);

    static void toMat8U(const std::vector<TDescriptor> &descriptors, cv::Mat &mat);
};

} // namespace DBoW2

#endif
