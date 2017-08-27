// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include <iostream>

#include <fstream>
#include <set>

#include "TestConfig.hpp"

#include "ApproxMVBB/ComputeApproxMVBB.hpp"

#include "CPUTimer.hpp"
#include "TestFunctions.hpp"

namespace ApproxMVBB
{
namespace MinAreaRectangleTest
{
template <typename TMatrix>
void minRectTest(std::string name, const TMatrix& v)
{
    using namespace PointFunctions;
    using namespace TestFunctions;

    dumpPointsMatrixBinary(getPointsDumpPath(name, ".bin"), v);
    dumpPointsMatrix(getPointsDumpPath(name, ".txt"), v);

    std::cout << "\n\nStart MinAreaRectangle Test " + name + "" << std::endl;
    START_TIMER(start)

    MinAreaRectangle c(v);
    c.compute();

    STOP_TIMER_SEC(count, start)
    std::cout << "Timings: " << count << " sec for " << v.cols() << " points" << std::endl;
    std::cout << "End MinAreaRectangle Test " + name + "" << std::endl;
    auto rect = c.getMinRectangle();

    Matrix2Dyn p(2, 6);
    p.col(0) = rect.m_p;
    p.col(1) = rect.m_p + rect.m_u * rect.m_uL;
    p.col(2) = rect.m_p + rect.m_u * rect.m_uL + rect.m_v * rect.m_vL;
    p.col(3) = rect.m_p + rect.m_v * rect.m_vL;
    p.col(4) = rect.m_u;
    p.col(5) = rect.m_v;

    dumpPointsMatrixBinary(getFileOutPath(name), p);
    // dumpPointsMatrix(getFileOutPath(name,".txt"),p);

    // Compare with validation file
    TMatrix valid = p;
    valid.setConstant(std::numeric_limits<PREC>::signaling_NaN());
    readPointsMatrixBinary(getFileValidationPath(name), valid);

    // Assert all cols of p are in valid
    EXPECT_TRUE(assertNearArrayColsRows<true>(p.leftCols(4), valid.leftCols(4))) << "Valid Points:" << std::endl
                                                                                 << valid.transpose() << std::endl
                                                                                 << " computed:" << std::endl
                                                                                 << p.transpose() << std::endl;
}
}
}

using namespace ApproxMVBB;
using namespace TestFunctions;
using namespace PointFunctions;
using namespace ApproxMVBB::MinAreaRectangleTest;

MY_TEST(MinAreaRectangleTest, PointsRandom10)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, PointsRandom10);
    // generate points
    Matrix2Dyn t(2, 10);
    t = t.unaryExpr(f);
    minRectTest(testName, t);
}

MY_TEST(MinAreaRectangleTest, UnitRectangle)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, UnitRectangle);

    Vector2List t;
    t.push_back(Vector2(0, 0));
    t.push_back(Vector2(1, 0));
    t.push_back(Vector2(1, 1));
    t.push_back(Vector2(0, 1));

    Matrix2Dyn v(2, t.size());
    for (unsigned int i = 0; i < t.size(); ++i)
    {
        v.col(i) = t[i];
    }
    minRectTest(testName, v);
}

MY_TEST(MinAreaRectangleTest, TwoPoints)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, TwoPoints);

    Vector2List v;
    v.push_back(Vector2(1, 0));
    v.push_back(Vector2(3, 3));
    ApproxMVBB::Matrix2Dyn t(2, v.size());
    for (unsigned int i = 0; i < v.size(); ++i)
    {
        t.col(i) = v[i];
    }
    minRectTest(testName, t);
}

MY_TEST(MinAreaRectangleTest, AlmostLine)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, AlmostLine);

    Vector2List v;
    v.push_back(Vector2(0, 0));
    v.push_back(Vector2(1, 1));
    v.push_back(Vector2(1, 1 + 1e-13));
    v.push_back(Vector2(2, 2));
    ApproxMVBB::Matrix2Dyn t(2, v.size());
    for (unsigned int i = 0; i < v.size(); ++i)
    {
        t.col(i) = v[i];
    }
    minRectTest(testName, t);
}

MY_TEST(MinAreaRectangleTest, Line3)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, Line3);

    Vector2List t;
    t.push_back(Vector2(0, 0));
    t.push_back(Vector2(1, 1));
    t.push_back(Vector2(2, 2));

    Matrix2Dyn v(2, t.size());
    for (unsigned int i = 0; i < t.size(); ++i)
    {
        v.col(i) = t[i];
    }
    minRectTest(testName, v);
}

MY_TEST(MinAreaRectangleTest, Line2)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, Line2);

    Vector2List t;
    t.push_back(Vector2(0, 0));
    t.push_back(Vector2(1, 1));

    Matrix2Dyn v(2, t.size());
    for (unsigned int i = 0; i < t.size(); ++i)
    {
        v.col(i) = t[i];
    }
    minRectTest(testName, v);
}

MY_TEST(MinAreaRectangleTest, Triangle)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, Triangle);

    Vector2List t;
    t.push_back(Vector2(0, 0));
    t.push_back(Vector2(1, 1));
    t.push_back(Vector2(1, -1));

    Matrix2Dyn v(2, t.size());
    for (unsigned int i = 0; i < t.size(); ++i)
    {
        v.col(i) = t[i];
    }
    minRectTest(testName, v);
}

MY_TEST(MinAreaRectangleTest, RandomTriangle)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, RandomTriangle);

    Matrix2Dyn v(2, 3);
    v = v.unaryExpr(f);
    minRectTest(testName, v);
}

MY_TEST(MinAreaRectangleTest, Point)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, Point);

    Vector2List t;
    t.push_back(Vector2(1, 0));

    Matrix2Dyn v(2, t.size());
    for (unsigned int i = 0; i < t.size(); ++i)
    {
        v.col(i) = t[i];
    }
    minRectTest(testName, v);
}

MY_TEST(MinAreaRectangleTest, PointsOnCricle1000)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, PointsOnCricle1000);

    unsigned int max = 1000;
    ApproxMVBB::Matrix2Dyn t(2, max);
    for (unsigned int i = 0; i < max; i++)
    {
        t.col(i) = Vector2(std::cos(0.0001 / max * i), std::sin(0.0001 / max * i));
    }
    minRectTest(testName, t);
}

MY_TEST(MinAreaRectangleTest, NoPoint)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, NoPoint);
    Matrix2Dyn v(2, 0);
    minRectTest(testName, v);
}

MY_TEST(MinAreaRectangleTest, Points2DRectFail)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, Points2DRectFail);

    auto t = getPointsFromFile2D(getFileInPath("PointsSimulation2DRectFail.txt"));
    ApproxMVBB::Matrix2Dyn v(2, t.size());
    for (unsigned int i = 0; i < t.size(); ++i)
    {
        v.col(i) = t[i];
    }
    minRectTest(testName, v);
}

#ifdef ApproxMVBB_TESTS_HIGH_PERFORMANCE
MY_TEST(MinAreaRectangleTest, PointsRandom10M)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, PointsRandom10M);
    // generate points
    ApproxMVBB::Matrix2Dyn t(2, 10000000);
    t = t.unaryExpr(f);
    minRectTest(testName, t);
}
#endif

MY_TEST(MinAreaRectangleTest, PointsBadProjection)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, PointsBadProjection);

    Matrix2Dyn t(2, 400);
    readPointsMatrixBinary(getFileInPath("PointsBadProjection.bin"), t, false);
    minRectTest(testName, t);
}
MY_TEST(MinAreaRectangleTest, PointsBadProjection2)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, PointsBadProjection2);

    Matrix2Dyn t(2, 400);
    readPointsMatrixBinary(getFileInPath("PointsBadProjection2.bin"), t, false);
    minRectTest(testName, t);
}
MY_TEST(MinAreaRectangleTest, PointsBadProjection3)
{
    MY_TEST_RANDOM_STUFF(MinAreaRectangleTest, PointsBadProjection3);

    Matrix2Dyn t(2, 400);
    readPointsMatrixBinary(getFileInPath("PointsBadProjection3.bin"), t, false);
    minRectTest(testName, t);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
