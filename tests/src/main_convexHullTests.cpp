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

#include "TestFunctions.hpp"
#include "CPUTimer.hpp"


namespace ApproxMVBB {

namespace ConvexHullTest {

    std::string genTestName(std::string name){
        return "ConvexHullTest-" + name;
    }

    template<typename T = PREC>
    std::string getPrecAbrev();

    template<>
    std::string getPrecAbrev<double>(){
        return "double";
    }
    template<>
    std::string getPrecAbrev<float>(){
        return "float";
    }

    std::string getFileInPath(std::string name){
        return ApproxMVBB_TESTS_INPUT_FILES_DIR "/" + name;
    }

    std::string getPointsDumpPath(std::string  name,std::string suffix=".bin"){
        return ApproxMVBB_TESTS_OUTPUT_FILES_DIR "/" + genTestName(name)+"-"+getPrecAbrev()+suffix;
    }

    std::string getFileOutPath(std::string name, std::string suffix=".bin"){
        return ApproxMVBB_TESTS_OUTPUT_FILES_DIR "/" + genTestName(name) +"-"+getPrecAbrev()+"-"+"Out"+suffix;
    }
    std::string getFileValidationPath(std::string name, std::string suffix=".bin"){
        return ApproxMVBB_TESTS_VALIDATION_FILES_DIR "/" + getPrecAbrev()+"/"+genTestName(name) +"-"+getPrecAbrev()+"-"+"Out"+suffix;
    }

    template<typename TMatrix>
    void convexHullTest(std::string name, const TMatrix & v, bool dumpPoints = true) {
        using namespace TestFunctions;
        using namespace PointFunctions;

        if(dumpPoints) {
            dumpPointsMatrixBinary( getPointsDumpPath(name,".bin") ,v);
            dumpPointsMatrix( getPointsDumpPath(name,".txt"),v);
        }
        std::cout << "\n\nStart ConvexHull Test "<< name << "" << std::endl;
        START_TIMER(start)
        ConvexHull2D c(v);
        c.compute();
        STOP_TIMER_SEC(count, start)
        std::cout << "Timings: " << count << " sec for " <<v.cols() << " points" << std::endl;
        std::cout << "End ConvexHull Test "<< name << "" << std::endl;
        if(!c.verifyHull()) {
            std::cerr << "ConvexHull Test "<< name << " not ok!" << std::endl;
        }


        auto ind = c.getIndices();
        std::cout << "ConvexHull Points: " << ind.size()  << std::endl;
        unsigned int j = 0;
        ApproxMVBB::Matrix2Dyn qHull(2,ind.size());
        for(auto & i : ind) {
            qHull.col(j++) = v.col(i);
        }


        dumpPointsMatrixBinary( getFileOutPath(name),qHull);

        // Compare with validation file
        TMatrix valid = qHull;
        valid.setConstant(std::numeric_limits<PREC>::signaling_NaN());
        readPointsMatrixBinary( getFileValidationPath(name) , valid);
        assertAlmostEqualArrays(qHull,valid);
    }

//    void test() {


};
};

using namespace ApproxMVBB;
using namespace TestFunctions;
using namespace PointFunctions;
using namespace ApproxMVBB::ConvexHullTest;

TEST(ConvexHullTest, PointsRandom10) {
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        // generate points
        Matrix2Dyn t(2,10);
        t = t.unaryExpr( f );
        convexHullTest("PointsRandom10",t);
}

TEST(ConvexHullTest, Line4) {


        Vector2List t;
        t.push_back(Vector2(0,0));
        t.push_back(Vector2(1,1));
        t.push_back(Vector2(2,2));
        t.push_back(Vector2(3,3));
        t.push_back(Vector2(-1,1));

        Matrix2Dyn v(2,t.size());
        for(unsigned int i = 0; i<t.size(); ++i) {
            v.col(i) = t[i];
        }
        convexHullTest("Line4",v);
}


TEST(ConvexHullTest, Line3) {

        Vector2List t;
        t.push_back(Vector2(0,0));
        t.push_back(Vector2(1,1));
        t.push_back(Vector2(2,2));

        Matrix2Dyn v(2,t.size());
        for(unsigned int i = 0; i<t.size(); ++i) {
            v.col(i) = t[i];
        }
        convexHullTest("Line3",v);
}

TEST(ConvexHullTest, Line_2) {

        Vector2List t;
        t.push_back(Vector2(0,0));
        t.push_back(Vector2(1,1));

        Matrix2Dyn v(2,t.size());
        for(unsigned int i = 0; i<t.size(); ++i) {
            v.col(i) = t[i];
        }
        convexHullTest("Line2",v);
}

TEST(ConvexHullTest, Triangle) {

        Vector2List t;
        t.push_back(Vector2(0,0));
        t.push_back(Vector2(1,1));
        t.push_back(Vector2(1,-1));

        Matrix2Dyn v(2,t.size());
        for(unsigned int i = 0; i<t.size(); ++i) {
            v.col(i) = t[i];
        }
        convexHullTest("Triangle",v);
}

TEST(ConvexHullTest, Point) {

        Vector2List t;
        t.push_back(Vector2(0,0));

        Matrix2Dyn v(2,t.size());
        for(unsigned int i = 0; i<t.size(); ++i) {
            v.col(i) = t[i];
        }
        convexHullTest("Point",v);
}

TEST(ConvexHullTest, PointsOnCricle1000) {

        unsigned int max = 1000;
        ApproxMVBB::Matrix2Dyn t(2,max);
        for(unsigned int i=0; i<max; i++) {
            t.col(i) = Vector2(std::cos(0.0001/max * i) ,std::sin(0.0001/max * i) );
        }
        convexHullTest("PointsOnCricle1000",t);
}

TEST(ConvexHullTest, Points2DRectFail) {

        auto t = getPointsFromFile2D(getFileInPath("PointsSimulation2DRectFail.txt"));
        ApproxMVBB::Matrix2Dyn v(2,t.size());
        for(unsigned int i = 0; i<t.size(); ++i) {
            v.col(i) = t[i];
        }
        convexHullTest("Points2DRectFail",v);
}

TEST(ConvexHullTest, PointsBadProjectionFilter) {

        Matrix2Dyn t(2,400);
        readPointsMatrixBinary(getFileInPath("PointsBadProjection.bin"),t,false);

        // Filter points
        std::set<unsigned int> i = {0,29,180,
                                    212,213,
                                    192,193,
                                    175,176,
                                    162,163,
                                    146,147,
                                    129,130,
                                    112,113,
                                    96,97,
                                    79,80,
                                    58,59,
                                    36,37,
                                    7,8,
                                    1,
                                    226,196,154,137,30,4
                                   };
        t = filterPoints(t,i);
        convexHullTest("PointsBadProjectionFilter",t);
}

TEST(ConvexHullTest, PointsBadProjection) {

        Matrix2Dyn t(2,400);
        readPointsMatrixBinary(getFileInPath("PointsBadProjection.bin"),t,false);
        convexHullTest("PointsBadProjection",t);
}
TEST(ConvexHullTest, PointsBadProjection2) {

        Matrix2Dyn t(2,400);
        readPointsMatrixBinary(getFileInPath("PointsBadProjection2.bin"),t,false);
        convexHullTest("PointsBadProjection2",t);
}
TEST(ConvexHullTest, PointsBadProjection3) {

        Matrix2Dyn t(2,400);
        readPointsMatrixBinary(getFileInPath("PointsBadProjection3.bin"),t,false);
        convexHullTest("PointsBadProjection3",t);
}
TEST(ConvexHullTest, PointsBadProjection4) {

        Matrix2Dyn t(2,16);
        t.setZero();
        readPointsMatrixBinary(getFileInPath("PointsBadProjection4.bin"),t,false);
        convexHullTest("PointsBadProjection4",t);
}
TEST(ConvexHullTest, PointsBadProjection5) {

        Matrix2Dyn t(2,5);
        t.setZero();
        readPointsMatrixBinary(getFileInPath("PointsBadProjection5.bin"),t,false);
        convexHullTest("PointsBadProjection5",t);
}
TEST(ConvexHullTest, PointsBadProjection6) {
        Matrix2Dyn t(2,100);
        t.setZero();
        readPointsMatrixBinary(getFileInPath("PointsBadProjection6.bin"),t,false);
        convexHullTest("PointsBadProjection6",t);
}

#ifdef ApproxMVBB_TESTS_HIGH_PERFORMANCE
TEST(ConvexHullTest, PointsRandom14M) {
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(-1.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        // generate points
        ApproxMVBB::Matrix2Dyn t(2,14000000);
        t = t.unaryExpr( f );
        convexHullTest("PointsRandom14M",t);
}
#endif




int main(int argc, char **argv) {
        srand48(314159);
        srand(314159);
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}
