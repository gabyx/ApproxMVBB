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
    namespace DiameterTest
    {
        template<typename TMatrix>
        void diameterTest(std::string name, const TMatrix& v, PREC epsilon = 0.001)
        {
            namespace pf = PointFunctions;
            namespace tf = TestFunctions;

            TMatrix in = v;
            std::cout << "\n\nStart diameterTest: " + name + "" << std::endl;
            START_TIMER(start)
            auto pp = pf::estimateDiameter<3>(v, epsilon);
            STOP_TIMER_SEC(count, start)
            std::cout << "Timings: " << count << " sec for " << v.cols() << " points" << std::endl;
            std::cout << "End diameterTest " + name << std::endl;

            Matrix3Dyn diam(3, 2);
            diam.col(0) = pp.first;
            diam.col(1) = pp.second;
            tf::dumpPointsMatrixBinary(tf::getFileOutPath(name, ".bin"), diam);  // dumps diameter
            tf::dumpPointsMatrixBinary(tf::getFileOutPath(name, "2.bin"), v);    // dump generated points

            // CHECK
            try
            {
                Matrix3Dyn diamV(3, 2);
                tf::readPointsMatrixBinary(tf::getFileValidationPath(name, ".bin"), diamV);
                Matrix3Dyn inputPointsV;
                tf::readPointsMatrixBinary(tf::getFileValidationPath(name, "2.bin"), inputPointsV);

                EXPECT_TRUE((in.array() == v.array()).all())
                    << " estimate diameter changed the point cloud! should not happed!";

                EXPECT_TRUE(tf::assertNearArrayColsRows(diam, diamV)) << "Diameter valid: " << std::endl
                                                                  << diamV << std::endl
                                                                  << "Diameter computed: " << std::endl
                                                                  << diam << std::endl;

                EXPECT_TRUE(tf::assertNearArray(v, inputPointsV)) << "input points not the same as valid ones";
            }
            catch(ApproxMVBB::Exception& e)
            {
                ASSERT_TRUE(false) << "Exception in checking inside test!: " << e.what() << std::endl;
            }
        }
    }  // namespace DiameterTest
}     // namespace ApproxMVBB

using namespace ApproxMVBB;
using namespace ApproxMVBB::DiameterTest;

MY_TEST(DiameterTest, RandomGenerator)
{
    MY_TEST_RANDOM_STUFF(DiameterTest, RandomGenerator);
    // generate points
    {
        std::cout << "Test: DefaultUniformUIntDistribution" << std::endl;
        MyMatrix::VectorDyn<unsigned int> t(30);
        RandomGenerators::DefaultRandomGen specialGen(RandomGenerators::defaultSeed);
        RandomGenerators::DefaultUniformUIntDistribution<unsigned int> specialUni(0, 30);
        auto f1 = [&](int) { return specialUni(specialGen); };
        t       = t.unaryExpr(f1);

        tf::dumpPointsMatrixBinary(tf::getFileOutPath(testName), t);

        // Check
        decltype(t) tvalid;
        tf::readPointsMatrixBinary(tf::getFileValidationPath(testName), tvalid);

        EXPECT_TRUE((t.array() == tvalid.array()).all()) << " vector valid: " << std::endl
                                                         << t.transpose() << "and: " << std::endl
                                                         << tvalid.transpose() << std::endl;
    }

    {
        std::cout << "Test: DefaultUniformRealDistribution" << std::endl;
        MyMatrix::VectorDyn<PREC> t(30);
        RandomGenerators::DefaultRandomGen specialGen(RandomGenerators::defaultSeed);
        RandomGenerators::DefaultUniformRealDistribution<PREC> specialUni(0, 30);
        auto f1 = [&](int) { return specialUni(specialGen); };
        t       = t.unaryExpr(f1);

        tf::dumpPointsMatrixBinary(tf::getFileOutPath(testName, "2.bin"), t);

        // Check
        decltype(t) tvalid;
        tf::readPointsMatrixBinary(tf::getFileValidationPath(testName, "2.bin"), tvalid);

        EXPECT_TRUE((t.array() == tvalid.array()).all()) << " vector valid: " << std::endl
                                                         << t.transpose() << "and: " << std::endl
                                                         << tvalid.transpose() << std::endl;
    }

    {
        std::cout << "Test: DefaultRandomGen" << std::endl;
        using T = RandomGenerators::DefaultRandomGen::result_type;
        MyMatrix::VectorDyn<T> t(30);
        RandomGenerators::DefaultRandomGen specialGen(RandomGenerators::defaultSeed);
        auto f1 = [&](T) { return specialGen(); };
        t       = t.unaryExpr(f1);

        tf::dumpPointsMatrixBinary(tf::getFileOutPath(testName, "3.bin"), t);
        // Check
        // Check
        decltype(t) tvalid;
        tf::readPointsMatrixBinary(tf::getFileValidationPath(testName, "3.bin"), tvalid);

        EXPECT_TRUE((t.array() == tvalid.array()).all()) << " vector valid: " << std::endl
                                                         << t.transpose() << "and: " << std::endl
                                                         << tvalid.transpose() << std::endl;
    }
}

MY_TEST(DiameterTest, PointsRandom3)
{
    MY_TEST_RANDOM_STUFF(DiameterTest, PointsRandom3);
    auto f = [&](PREC) { return uni(rng); };
    Matrix3Dyn t(3, 3);
    t = t.unaryExpr(f);
    diameterTest(testName, t);
}

MY_TEST(DiameterTest, PointsRandom500)
{
    MY_TEST_RANDOM_STUFF(DiameterTest, PointsRandom500);
    auto f = [&](PREC) { return uni(rng); };
    Matrix3Dyn t(3, 500);
    t = t.unaryExpr(f);
    diameterTest(testName, t);
}

MY_TEST(DiameterTest, PointsRandom10000)
{
    MY_TEST_RANDOM_STUFF(DiameterTest, PointsRandom10000);
    auto f = [&](PREC) { return uni(rng); };
    Matrix3Dyn t(3, 10000);
    t = t.unaryExpr(f);
    diameterTest(testName, t);
}

MY_TEST(DiameterTest, UnitCube)
{
    MY_TEST_RANDOM_STUFF(DiameterTest, UnitCube);
    Matrix3Dyn t(3, 8);
    t.col(0) = ApproxMVBB::Vector3(0, 0, 0);
    t.col(1) = ApproxMVBB::Vector3(1, 0, 0);
    t.col(2) = ApproxMVBB::Vector3(1, 1, 0);
    t.col(3) = ApproxMVBB::Vector3(0, 1, 0);
    t.col(4) = ApproxMVBB::Vector3(0, 0, 1);
    t.col(5) = ApproxMVBB::Vector3(1, 0, 1);
    t.col(6) = ApproxMVBB::Vector3(1, 1, 1);
    t.col(7) = ApproxMVBB::Vector3(0, 1, 1);

    diameterTest(testName, t);
}

// MY_TEST(DiameterTest, PointsSimulation) {
// MY_TEST_RANDOM_STUFF(PointsSimulation)
//        auto v = tf::getPointsFromFile3D(tf::getFileInPath("PointsSimulation.txt"));
//
//        Matrix3Dyn t(3,v.size());
//        for(unsigned int i = 0; i<v.size(); ++i) {
//                t.col(i) = v[i];
//        }
//        pf::applyRandomRotTrans(t,f);
//        std::cout << "Applied Transformation" << std::endl;
//        diameterTest(testName,t);
//}
//
// MY_TEST(DiameterTest, PointsSimulationFailMVBB) {
// MY_TEST_RANDOM_STUFF(PointsSimulationFailMVBB)
//        auto v = tf::getPointsFromFile3D(tf::getFileInPath("PointsSimulationFailMVBB.txt"));
//        Matrix3Dyn t(3,v.size());
//        for(unsigned int i = 0; i<v.size(); ++i) {
//        t.col(i) = v[i];
//        }
//        pf::applyRandomRotTrans(t,f);
//        std::cout << "Applied Transformation" << std::endl;
//        diameterTest(testName,t);
//}
//
// MY_TEST(DiameterTest, Bunny) {
// MY_TEST_RANDOM_STUFF(Bunny)
//        auto v = tf::getPointsFromFile3D(tf::getFileInPath("Bunny.txt"));
//        Matrix3Dyn t(3,v.size());
//        for(unsigned int i = 0; i<v.size(); ++i) {
//                t.col(i) = v[i];
//        }
//        pf::applyRandomRotTrans(t,f);
//        std::cout << "Applied Transformation" << std::endl;
//        diameterTest(testName,t);
//}
//
//#ifdef ApproxMVBB_TESTS_HIGH_PERFORMANCE
// MY_TEST(DiameterTest, PointsRandom14M) {
// MY_TEST_RANDOM_STUFF(PointsRandom14M)
//        Matrix3Dyn t(3,140000000);
//        t = t.unaryExpr( f );
//        diameterTest(testName,t);
//}
//
//
// MY_TEST(DiameterTest, Lucy) {
// MY_TEST_RANDOM_STUFF(Lucy)
//        auto v = tf::getPointsFromFile3D(getFileInAddPath("Lucy.txt"));
//        Matrix3Dyn t(3,v.size());
//        for(unsigned int i = 0; i<v.size(); ++i) {
//            t.col(i) = v[i];
//        }
//        pf::applyRandomRotTrans(t,f);
//        diameterTest(testName,t);
//}
//#endif

MY_TEST(DiameterTest, Plane)
{
    MY_TEST_RANDOM_STUFF(DiameterTest, Plane);
    auto f = [&](PREC) { return uni(rng); };
    Matrix3Dyn t(3, 3);
    t = t.unaryExpr(f);
    diameterTest(testName, t, true);
}

// MY_TEST(DiameterTest, PointClouds) {
// MY_TEST_RANDOM_STUFF(PointClouds)
//
//        for(unsigned int k=0; k<51; k++) {
//            auto v = tf::getPointsFromFile3D(tf::getFileInPath("PointCloud_" + std::to_string(k) +".txt"));
//            Matrix3Dyn t(3,v.size());
//            for(unsigned int i = 0; i<v.size(); ++i) {
//                t.col(i) = v[i];
//            }
//
//            pf::applyRandomRotTrans(t,f);
//            diameterTest("PointClouds-Nr"+std::to_string(k),t);
//        }
//}

MY_TEST(DiameterTest, UnitPatches2D)
{
    MY_TEST_RANDOM_STUFF(DiameterTest, UnitPatches2D);
    auto f = [&](PREC) { return uni(rng); };
    for(int i = 0; i < 10; ++i)
    {
        ApproxMVBB::Matrix3Dyn t(3, 500);
        t = t.unaryExpr(f);
        t.row(2).setZero();
        diameterTest(testName + "-Nr-" + std::to_string(i), t);
    }
}

// MY_TEST(DISABLED_DiameterTest, Plane) {
// MY_TEST_RANDOM_STUFF(Plane)
//
//        Matrix3Dyn t(3,3);
//        t.setConstant(std::numeric_limits<PREC>::signaling_NaN());
//        diameterTest(testName,t);
//}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
