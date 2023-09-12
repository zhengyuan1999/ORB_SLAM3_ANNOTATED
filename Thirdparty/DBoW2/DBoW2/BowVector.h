/**
 * File: BowVector.h
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: bag of words vector
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_BOW_VECTOR__
#define __D_T_BOW_VECTOR__

#include <iostream>
#include <map>
#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/map.hpp>

namespace DBoW2
{

typedef unsigned int WordId; // 单词 ID 类型定义（Id of words）
typedef double WordValue; 	 // 权重类型定义（Value of a word）
typedef unsigned int NodeId; // 节点 ID 类型定义（Id of nodes in the vocabulary treee）

// 归一化方式（L-norms for normalization）
enum LNorm
{
    L1,
    L2
};

// 权重类型（Weighting type）
enum WeightingType
{
    TF_IDF,
    TF,
    IDF,
    BINARY
};

// 评分标准（Scoring type）
enum ScoringType
{
    L1_NORM,
    L2_NORM,
    CHI_SQUARE,
    KL,
    BHATTACHARYYA,
    DOT_PRODUCT,
};

// 该类继承自 std::map<unsigned int, unsigned int>，保证了该类有序且关键字唯一
class BowVector : public std::map<WordId, WordValue> // 图像描述向量类（Vector of words to represent images）
{
    // 侵入式序列化
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const int version)
    {
        ar & boost::serialization::base_object<std::map<WordId, WordValue> >(*this);
    }

public:
    /**
     * @brief 什么也没做（Constructor）
     */
    BowVector(void);

    /**
     * @brief 什么也没做（Destructor）
     */
    ~BowVector(void);

    /**
     * @brief 如果单词存在（于本描述向量）则直接增加值，否则创建新的键值对 <id, v>（Adds
     * a value to a word value existing in the vector, or creates a new word
     * with the given value）
     * 
     * @param[in] id 单词 ID（word id to look for）
     * @param[in] v 待增加的权重（value to create the word with, or to add to existing word）
    */
    void addWeight(WordId id, WordValue v);

    /**
     * @brief 如果单词存在（于本描述向量）则增加值，否则不进行操作（Adds a word
     * with a value to the vector only if this does not exist yet）
     * 
     * @param[in] id 单词 ID（word id to look for）
     * @param[in] v 待增加的权重（value to create the word with, or to add to existing word）
    */
    void addIfNotExist(WordId id, WordValue v);

    /**
     * @brief 归一化该描述向量（L1-Normalizes the values in the vector）
     * 
     * @param[in] norm_type 归一化方式（norm used）
    */
    void normalize(LNorm norm_type);

    /**
     * @brief 打印描述向量的内容（Prints the content of the bow vector）
     * 
     * @param[in] out 输出流（stream）
     * @param[in] v 描述向量
     * 
     * @return 输出流
     */
    friend std::ostream &operator<<(std::ostream &out, const BowVector &v);

    /**
     * @brief 保存描述向量到一个 Matlab 格式文件中（Saves the bow
     * vector as a vector in a matlab file）
     * 
     * @param[in] filename 文件名
     * @param[in] W 词典中的单词数量（number of words in the vocabulary）
     */
    void saveM(const std::string &filename, size_t W) const;
};

} // namespace DBoW2

#endif
