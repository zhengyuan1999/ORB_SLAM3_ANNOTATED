/**
 * File: FeatureVector.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_FEATURE_VECTOR__
#define __D_T_FEATURE_VECTOR__

#include "BowVector.h"
#include <map>
#include <vector>
#include <iostream>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/map.hpp>

namespace DBoW2
{

/**
 * @brief 图像特征向量类（Vector of nodes with indexes of local features）
 * 
 * @note 正向索引，为了加速两图像帧特征点（的描述子）匹配（逆向索引而是为了加速计算图像相似度，被 ORB_SLAM3::KeyFrameDatabase 维护）
*/
// NodeId：节点 ID；std::vector<unsigned int>：保存图像特征点中属于 key（NodeId）的索引
class FeatureVector : public std::map<NodeId, std::vector<unsigned int> >
{
    // 侵入式序列化
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const int version)
    {
        ar & boost::serialization::base_object<std::map<NodeId, std::vector<unsigned int> > >(*this);
    }

public:
    /**
     * @brief 什么也没做（Constructor）
     */
    FeatureVector(void);

    /**
     * @brief 什么也没做（Destructor）
     */
    ~FeatureVector(void);

    /**
     * @brief 添加图像特征点索引到节点 ID 为 id 的节点（Adds a feature to an existing node,
     * or adds a new node with an initial feature）
     * 
     * @param[in] id 需要增加特征点索引的节点 ID（node id to add or to modify）
     * @param[in] i_feature 特征点在图像关键点向量中的索引位置（index of feature to add to the given node）
    */
    void addFeature(NodeId id, unsigned int i_feature);

    /**
     * @brief 打印特征向量的内容（Sends a string versions of the feature vector through the stream）
     * 
     * @param[in] out 输出流（stream）
     * @param[in] v 特征向量
     * 
     * @return 输出流
     */
    friend std::ostream &operator<<(std::ostream &out, const FeatureVector &v);
};

} // namespace DBoW2

#endif
