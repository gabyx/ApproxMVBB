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
    namespace DiameterOOBBTest
    {
        template<typename TMatrix>
        void diameterTest(std::string name,
                          const TMatrix& v,
                          bool dump                 = true,
                          unsigned int optLoops     = 10,
                          PREC epsilon              = 0.001,
                          unsigned int samplePoints = 400,
                          bool checkVolume          = true)
        {
            namespace pf = PointFunctions;
            namespace tf = TestFunctions;

            if(dump)
            {
                tf::dumpPointsMatrixBinary(tf::getPointsDumpPath(name, ".bin"), v);
                tf::dumpPointsMatrix(tf::getPointsDumpPath(name, ".txt"), v);
            }

            std::cout << "\n\nStart diameterOOBB test " + name + "" << std::endl;
            START_TIMER(start)
            auto oobb = ApproxMVBB::approximateMVBBDiam(v, epsilon, optLoops);
            STOP_TIMER_SEC(count, start)
            std::cout << "Timings: " << count << " sec for " << v.cols() << " points" << std::endl;
            std::cout << "End diameterOOBB test " + name << std::endl;

            oobb.expand(1e-10);
            if(!tf::checkPointsInOOBB(v, oobb))
            {
                std::cout << "WARNING: Not all points in OOBB.expand(1e-10)" << std::endl;
            }
            else
            {
                std::cout << "All points in OOBB!" << std::endl;
            }

            tf::dumpOOBB(tf::getFileOutPath(name, ".txt"), oobb);

            // Check OOBB
            OOBB validOOBB;
            try
            {
                validOOBB = tf::readOOBBAndCheck(oobb, tf::getFileValidationPath(name, ".txt"));
                if(checkVolume)
                {
                    ASSERT_GT(oobb.volume(), 1e-6) << "Volume too small: " << oobb.volume() << std::endl;
                }
            }
            catch(ApproxMVBB::Exception& e)
            {
                ASSERT_TRUE(false) << "Exception in checking inside test!: " << e.what() << std::endl;
            }

            // Sample with valid OOBB (computed OOBB might different R_IK but same corners points)
            std::cout << "Start Sampling test " + name + "" << std::endl;
            Matrix3Dyn sampled;
            START_TIMER(start2)
            ApproxMVBB::samplePointsGrid(sampled, v, samplePoints, validOOBB);
            STOP_TIMER_SEC(count2, start2)
            std::cout << "Timings: " << count2 << " sec for " << sampled.cols() << " points" << std::endl;
            std::cout << "End Sampling test " + name << std::endl;

            tf::dumpPointsMatrixBinary(tf::getFileOutPath(name, "2.bin"), sampled);

            Matrix3Dyn valid;
            try
            {
                // Check Sampled points
                Matrix3Dyn valid = sampled;
                valid.setConstant(std::numeric_limits<PREC>::signaling_NaN());
                tf::readPointsMatrixBinary(tf::getFileValidationPath(name, "2.bin"), valid);

                if(sampled.cols() > 100)
                {
                    EXPECT_TRUE(tf::assertNearArray(sampled, valid));
                }
                else
                {
                    EXPECT_TRUE(tf::assertNearArray(sampled, valid)) << std::endl
                                                                 << "valid points:" << valid.transpose() << std::endl
                                                                 << "and:" << std::endl
                                                                 << sampled.transpose();
                }
            }
            catch(ApproxMVBB::Exception& e)
            {
                ASSERT_TRUE(false) << "Exception in checking inside test!: " << e.what() << std::endl;
            }
        }
    };  // namespace DiameterOOBBTest
};      // namespace ApproxMVBB

using namespace ApproxMVBB;
using namespace ApproxMVBB::DiameterOOBBTest;

MY_TEST(DiameterOOBBTest, PointsRandom3)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, PointsRandom3);
    auto f = [&](PREC) { return uni(rng); };
    // generate points
    Matrix3Dyn t(3, 3);
    t = t.unaryExpr(f);
    diameterTest(testName, t, true, 10, 0.001, 3, false);
}

MY_TEST(DiameterOOBBTest, PointsRandom500)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, PointsRandom500);
    auto f = [&](PREC) { return uni(rng); };
    // generate points
    Matrix3Dyn t(3, 500);
    t = t.unaryExpr(f);
    diameterTest(testName, t);
}

MY_TEST(DiameterOOBBTest, PointsRandom10000)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, PointsRandom10000);
    auto f = [&](PREC) { return uni(rng); };
    // generate points
    Matrix3Dyn t(3, 10000);
    t = t.unaryExpr(f);
    diameterTest(testName, t);
}

MY_TEST(DiameterOOBBTest, UnitCube)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, UnitCube);
    Matrix3Dyn t(3, 8);
    t.col(0) = ApproxMVBB::Vector3(0, 0, 0);
    t.col(1) = ApproxMVBB::Vector3(1, 0, 0);
    t.col(2) = ApproxMVBB::Vector3(1, 1, 0);
    t.col(3) = ApproxMVBB::Vector3(0, 1, 0);
    t.col(4) = ApproxMVBB::Vector3(0, 0, 1);
    t.col(5) = ApproxMVBB::Vector3(1, 0, 1);
    t.col(6) = ApproxMVBB::Vector3(1, 1, 1);
    t.col(7) = ApproxMVBB::Vector3(0, 1, 1);

    diameterTest(testName, t, true, 1, 0.001, 4);
}

MY_TEST(DiameterOOBBTest, PointsSimulation)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, PointsSimulation);
    auto f = [&](PREC) { return uni(rng); };
    auto v = tf::getPointsFromFile3D(tf::getFileInPath("PointsSimulation.txt"));

    Matrix3Dyn t(3, v.size());
    for(unsigned int i = 0; i < v.size(); ++i)
    {
        t.col(i) = v[i];
    }
    pf::applyRandomRotTrans(t, f);
    std::cout << "Applied Transformation" << std::endl;
    diameterTest(testName, t);
}

MY_TEST(DiameterOOBBTest, PointsSimulationFailMVBB)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, PointsSimulationFailMVBB);
    auto f = [&](PREC) { return uni(rng); };

    auto v = tf::getPointsFromFile3D(tf::getFileInPath("PointsSimulationFailMVBB.txt"));
    Matrix3Dyn t(3, v.size());
    for(unsigned int i = 0; i < v.size(); ++i)
    {
        t.col(i) = v[i];
    }
    pf::applyRandomRotTrans(t, f);
    std::cout << "Applied Transformation" << std::endl;
    diameterTest(testName, t, true, 10);
}

MY_TEST(DiameterOOBBTest, Bunny)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, Bunny);
    auto f = [&](PREC) { return uni(rng); };

    auto v = tf::getPointsFromFile3D(tf::getFileInPath("Bunny.txt"));
    Matrix3Dyn t(3, v.size());
    for(unsigned int i = 0; i < v.size(); ++i)
    {
        t.col(i) = v[i];
    }
    pf::applyRandomRotTrans(t, f);
    std::cout << "Applied Transformation" << std::endl;
    diameterTest(testName, t, false, 10, 1);
}

#ifdef ApproxMVBB_TESTS_HIGH_PERFORMANCE
MY_TEST(DiameterOOBBTest, PointsRandom14M)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, PointsRandom14M);
    auto f = [&](PREC) { return uni(rng); };
    Matrix3Dyn t(3, 140000000);
    t = t.unaryExpr(f);
    diameterTest(testName, t, false, 0, 0.01);
}

MY_TEST(DiameterOOBBTest, Lucy)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, Lucy);
    auto f = [&](PREC) { return uni(rng); };
    auto v = tf::getPointsFromFile3D(getFileInAddPath("Lucy.txt"));
    Matrix3Dyn t(3, v.size());
    for(unsigned int i = 0; i < v.size(); ++i)
    {
        t.col(i) = v[i];
    }
    pf::applyRandomRotTrans(t, f);
    diameterTest(testName, t, false, 3, 100);
}
#endif

MY_TEST(DiameterOOBBTest, Plane)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, Plane);
    auto f = [&](PREC) { return uni(rng); };
    Matrix3Dyn t(3, 3);
    t = t.unaryExpr(f);
    diameterTest(testName, t, true, 10, 0.001, 2, false);
}

MY_TEST(DiameterOOBBTest, PointClouds)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, PointClouds);
    auto f = [&](PREC) { return uni(rng); };
    for(unsigned int k = 0; k < 51; k++)
    {
        auto v = tf::getPointsFromFile3D(tf::getFileInPath("PointCloud_" + std::to_string(k) + ".txt"));
        Matrix3Dyn t(3, v.size());
        for(unsigned int i = 0; i < v.size(); ++i)
        {
            t.col(i) = v[i];
        }

        pf::applyRandomRotTrans(t, f);
        diameterTest(testName + "-Nr" + std::to_string(k), t, true, 4, 0.1);
    }
}

MY_TEST(DiameterOOBBTest, UnitPatches2D)
{
    MY_TEST_RANDOM_STUFF(DiameterOOBBTest, UnitPatches2D);
    auto f = [&](PREC) { return uni(rng); };
    
    for(int i = 0; i < 10; ++i)
    {
        ApproxMVBB::Matrix3Dyn t(3, 500);
        t = t.unaryExpr(f);
        t.row(2).setZero();
        diameterTest(testName + "-Nr-" + std::to_string(i), t, true, 10, 0.0, 400, false);
    }
}

// MY_TEST(DISABLED_DiameterOOBBTest, Plane) {
// MY_TEST_RANDOM_STUFF(Plane)
//
//        Matrix3Dyn t(3,3);
//        t.setConstant(std::numeric_limits<PREC>::signaling_NaN());
//        diameterTest(testName,t,true,10,0.001,2);
//}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
