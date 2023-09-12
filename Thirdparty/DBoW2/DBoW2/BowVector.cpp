/**
 * File: BowVector.cpp
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: bag of words vector
 * License: see the LICENSE.txt file
 *
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "BowVector.h"

namespace DBoW2
{

// --------------------------------------------------------------------------

BowVector::BowVector(void) {}

// --------------------------------------------------------------------------

BowVector::~BowVector(void) {}

// --------------------------------------------------------------------------

void BowVector::addWeight(WordId id, WordValue v)
{
    /**
     * upper_bound：返回对象序列中第一个关键字“大于”查找关键字的迭代器
     * lower_bound：返回对象序列中第一个关键字“大于等于”查找关键字的迭代器
     * 实例：
     *      this->upper_bound(3)
     *                ↓
     *       1, 2, 3, 4, 5, 6, 7
     *             ↑
     *   this->lower_bound(3)
    */
    BowVector::iterator vit = this->lower_bound(id);

    // map 对象的 key_comp() 函数返回一个可调用对象（仿函数），其与 map 对象的排序规则一致（默认 std::less()）
    if (vit != this->end() && !(this->key_comp()(id, vit->first)))
    {
        vit->second += v;
    }
    else // 直接访问不就可以了？为什么要这么做？
    {
        this->insert(vit, BowVector::value_type(id, v));
    }
}

// --------------------------------------------------------------------------

void BowVector::addIfNotExist(WordId id, WordValue v)
{
    BowVector::iterator vit = this->lower_bound(id);

    if (vit == this->end() || (this->key_comp()(id, vit->first)))
    {
        this->insert(vit, BowVector::value_type(id, v));
    }
}

// --------------------------------------------------------------------------

void BowVector::normalize(LNorm norm_type)
{
    double norm = 0.0;
    BowVector::iterator it;

// 计算范数
    if (norm_type == DBoW2::L1) // L1 归一化
    {
        for (it = begin(); it != end(); ++it)
        {
            norm += fabs(it->second);
        }
    }
    else // L2 归一化
    {
        for (it = begin(); it != end(); ++it)
        {
            norm += it->second * it->second;
        }
        norm = sqrt(norm);
    }

// 归一化
    if (norm > 0.0)
    {
        for (it = begin(); it != end(); ++it)
        {
            it->second /= norm;
        }
    }
}

// --------------------------------------------------------------------------

std::ostream &operator<<(std::ostream &out, const BowVector &v)
{
    BowVector::const_iterator vit;
    // std::vector<unsigned int>::const_iterator iit; // 没用上
    unsigned int i = 0;
    const unsigned int N = v.size();
    for (vit = v.begin(); vit != v.end(); ++vit, ++i)
    {
        out << "<" << vit->first << ", " << vit->second << ">";

        if (i < N - 1)
        {
            out << ", ";
        }
    }

    return out;
}

// --------------------------------------------------------------------------

void BowVector::saveM(const std::string &filename, size_t W) const
{
    std::fstream f(filename.c_str(), std::ios::out); // 截断方式

    WordId last = 0;
    BowVector::const_iterator bit;
    for (bit = this->begin(); bit != this->end(); ++bit)
    {
        for (; last < bit->first; ++last)
        {
            f << "0 ";
        }
        f << bit->second << " ";

        last = bit->first + 1;
    }

    for (; last < (WordId)W; ++last)
    {
        f << "0 ";
    }

    f.close();
}

// --------------------------------------------------------------------------

} // namespace DBoW2
