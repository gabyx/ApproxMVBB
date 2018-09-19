// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================
#ifndef TestFunctions_hpp
#define TestFunctions_hpp

#include <functional>
#include <gtest/gtest.h>

#include "ApproxMVBB/Common/AssertionDebug.hpp"
#include "ApproxMVBB/Common/SfinaeMacros.hpp"
#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE

#include "ApproxMVBB/OOBB.hpp"
#include "ApproxMVBB/PointFunctions.hpp"
#include "ApproxMVBB/RandomGenerators.hpp"

#include "CommonFunctions.hpp"

#define MY_TEST(name1, name2) TEST(name1, name2)
#define MY_TEST_RANDOM_STUFF(name1, name2)                                            \
    using namespace ApproxMVBB;                                                       \
	namespace pf = ApproxMVBB::PointFunctions;                                        \
	namespace tf = ApproxMVBB::TestFunctions;                                         \
	std::string testName = #name1 "-" #name2;                                         \
    auto seed            = TestFunctions::hashString(#name2);                         \
    std::cout << "Seed for this test: " << seed << std::endl;                         \
    ApproxMVBB::RandomGenerators::DefaultRandomGen rng(seed);                         \
    ApproxMVBB::RandomGenerators::DefaultUniformRealDistribution<PREC> uni(0.0, 1.0); \
    auto f = [&](PREC) { return uni(rng); };

namespace ApproxMVBB
{
    namespace TestFunctions
    {
        ApproxMVBB_DEFINE_MATRIX_TYPES;
        ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES;

        template<typename A, typename B>
        ::testing::AssertionResult assertNearArray(const A& a, const B& b, PREC absError = 1e-6)
        {
            if(a.size() != b.size())
            {
                return ::testing::AssertionFailure() << "not same size";
            }
            if(a.rows() != b.rows())
            {
                return ::testing::AssertionFailure() << "not same rows";
            }
            if(((a - b).array().abs() >= absError).any())
            {
                return ::testing::AssertionFailure() << "not near: absTol:" << absError;
            }
            return ::testing::AssertionSuccess();
        }

        template<bool matchCols, typename A, typename B, ApproxMVBB_SFINAE_ENABLE_IF(matchCols == true)>
        ::testing::AssertionResult assertNearArrayColsRows_cr(const A& a, std::size_t i, const B& b, std::size_t j)
        {
            return assertNearArray(a.col(i), b.col(j));
        }
        template<bool matchCols, typename A, typename B, ApproxMVBB_SFINAE_ENABLE_IF(matchCols == false)>
        ::testing::AssertionResult assertNearArrayColsRows_cr(const A& a, std::size_t i, const B& b, std::size_t j)
        {
            return assertNearArray(a.row(i), b.row(j));
        }

        template<bool matchCols = true, typename A, typename B>
        ::testing::AssertionResult assertNearArrayColsRows(const A& a, const B& b)
        {
            if(a.size() != b.size())
            {
                return ::testing::AssertionFailure() << "not same size";
            }
            if(a.rows() != b.rows())
            {
                return ::testing::AssertionFailure() << "not same rows";
            }

            // Check all points
            std::vector<bool> indexMatched;
            if(matchCols)
            {
                indexMatched.resize(a.cols(), false);
            }
            else
            {
                indexMatched.resize(a.rows(), false);
            }
            auto s               = indexMatched.size();
            std::size_t nMatched = 0;
            std::size_t i        = 0;
            std::size_t pointIdx = 0;
            while(pointIdx < s)
            {
                if(i < s)
                {
                    // check points[pointIdx] against i-th valid one
                    if(!indexMatched[i] && assertNearArrayColsRows_cr<matchCols>(a, pointIdx, b, i))
                    {
                        indexMatched[i] = true;
                        ++nMatched;
                    }
                    else
                    {
                        ++i;
                        continue;
                    }
                }
                // all indices i checked go to next point, reset check idx
                i = 0;
                ++pointIdx;
            }

            if(nMatched != s)
            {
                return ::testing::AssertionFailure() << "Matched only " << nMatched << "/" << s;
            }
            return ::testing::AssertionSuccess();
        }

        template<typename Derived, typename IndexSet>
        Derived filterPoints(MatrixBase<Derived>& v, IndexSet& s)
        {
            Derived ret;
            ret.resize(v.rows(), s.size());

            auto size        = v.cols();
            decltype(size) k = 0;
            for(auto i : s)
            {
                if(i < size && i >= 0)
                {
                    ret.col(k++) = v.col(i);
                }
            };
            return ret;
        }

        template<typename Derived>
        bool checkPointsInOOBB(const MatrixBase<Derived>& points, OOBB oobb)
        {
            Matrix33 A_KI = oobb.m_q_KI.matrix();
            A_KI.transposeInPlace();

            Vector3 p;
            bool allInside   = true;
            auto size        = points.cols();
            decltype(size) i = 0;
            while(i < size && allInside)
            {
                p = A_KI * points.col(i);
                allInside &= (p(0) >= oobb.m_minPoint(0) && p(0) <= oobb.m_maxPoint(0) && p(1) >= oobb.m_minPoint(1) &&
                              p(1) <= oobb.m_maxPoint(1) && p(2) >= oobb.m_minPoint(2) && p(2) <= oobb.m_maxPoint(2));
                ++i;
            }
            return allInside;
        }
    }  // namespace TestFunctions
}  // namespace ApproxMVBB

#endif
