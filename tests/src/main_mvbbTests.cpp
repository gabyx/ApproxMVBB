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

namespace MVBBTests {

    std::string genTestName( std::string name){
        return "MVBBTest-" + name;
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

    std::string getFileInAddPath(std::string name){
        return ApproxMVBB_TESTS_INPUT_FILES_ADD_DIR "/" + name;
    }

    std::string getPointsDumpPath(std::string name,std::string suffix=".bin"){
        return ApproxMVBB_TESTS_OUTPUT_FILES_DIR "/" + genTestName(name)+"-"+getPrecAbrev()+suffix;
    }

    std::string getFileOutPath(std::string name, std::string suffix=".bin"){
        return ApproxMVBB_TESTS_OUTPUT_FILES_DIR "/" + genTestName(name) +"-"+getPrecAbrev()+"-"+"Out"+suffix;
    }
    std::string getFileValidationPath(std::string name, std::string suffix=".bin"){
        return ApproxMVBB_TESTS_VALIDATION_FILES_DIR "/" + getPrecAbrev()+"/"+genTestName(name) +"-"+getPrecAbrev()+"-"+"Out"+suffix;
    }

    template<typename TMatrix>
    void mvbbTest(std::string name, const TMatrix & v,
                  bool dumpPoints = true,
                  PREC eps = 0.001,
                  unsigned int nPoints = 400,
                  unsigned int gridSize = 5,
                  unsigned int mvbbDiamOptLoops = 2,
                  unsigned int gridSearchOptLoops = 10
                 ) {
        using namespace PointFunctions;
        using namespace TestFunctions;

        setRandomSeedStd(name);

        if(dumpPoints) {
                dumpPointsMatrixBinary( getPointsDumpPath(name,".bin") ,v);
                dumpPointsMatrix( getPointsDumpPath(name,".txt"),v);
        }

        std::cout << "\n\nStart MVBBTest Test "+ name +"" << std::endl;
        START_TIMER(start)
        auto oobb = ApproxMVBB::approximateMVBB(v,eps,nPoints,gridSize,mvbbDiamOptLoops,gridSearchOptLoops,std::mt19937(314159));
        STOP_TIMER_SEC(count, start)
        std::cout << "Timings: " << count << " sec for " <<v.cols() << " points" << std::endl;
        std::cout << "End MVBBTest "+ name << std::endl;



        oobb.expandToMinExtentRelative(0.1);

        // Make all points inside OOBB!
        Matrix33 A_KI = oobb.m_q_KI.matrix().transpose(); // faster to store the transformation matrix first
        auto size = v.cols();
        for( unsigned int i=0;  i<size; ++i ) {
            oobb.unite(A_KI*v.col(i));
        }

        dumpOOBB(getFileOutPath(name ,".txt"), oobb);

        // Check
        try{
            Vector3 minP;
            Vector3 maxP;
            Matrix33 R_KI;
            readOOBB( getFileValidationPath(name,".txt") , minP, maxP, R_KI);
            std::cout <<"valid minP: " <<  minP.transpose() << " and " << oobb.m_minPoint.transpose() << std::endl;
            std::cout <<"valid maxP: " << maxP.transpose() << " and " << oobb.m_maxPoint.transpose() << std::endl;
            std::cout <<"valid R_IK: " << oobb.m_q_KI.matrix()  << " and " << std::endl << R_KI << std::endl;
            assertAlmostEqualArrays(oobb.m_minPoint,minP);
            assertAlmostEqualArrays(oobb.m_maxPoint,maxP);
            assertAlmostEqualArrays(oobb.m_q_KI.matrix(),R_KI);

            ASSERT_GT(oobb.volume() , 1e-6)  << "Volume too small: " << oobb.volume() << std::endl;
        }
        catch( ApproxMVBB::Exception & e){
            ASSERT_TRUE(false) << "Exception in checking inside test!"  << e.what() << std::endl;
        }

    }

};
};

using namespace ApproxMVBB;
using namespace TestFunctions;
using namespace PointFunctions;
using namespace ApproxMVBB::MVBBTests;


TEST(MVBBTest, PointsRandom1) {
        static std::mt19937 rng(TestFunctions::randomSeed);
        static std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        // generate points
        Matrix3Dyn t(3,1);
        t = t.unaryExpr( f );
        mvbbTest("PointsRandom1",t,true,0.001,1,5,10,10);
}

TEST(MVBBTest, PointsRandom2) {
        static std::mt19937 rng(TestFunctions::randomSeed);
        static std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        // generate points
        Matrix3Dyn t(3,2);
        t = t.unaryExpr( f );
        mvbbTest("PointsRandom2",t,true,0.001,2,5,10,10);
}

TEST(MVBBTest, PointsRandom3) {
        static std::mt19937 rng(TestFunctions::randomSeed);
        static std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        // generate points
        Matrix3Dyn t(3,2);
        t = t.unaryExpr( f );
        mvbbTest("PointsRandom2",t,true,0.001,3,5,10,10);
}

TEST(MVBBTest, UnitCube) {
        ApproxMVBB::Matrix3Dyn t(3,8);
        t.col(0) = ApproxMVBB::Vector3(0,0,0);
        t.col(1) = ApproxMVBB::Vector3(1,0,0);
        t.col(2) = ApproxMVBB::Vector3(1,1,0);
        t.col(3) = ApproxMVBB::Vector3(0,1,0);
        t.col(4) = ApproxMVBB::Vector3(0,0,1);
        t.col(5) = ApproxMVBB::Vector3(1,0,1);
        t.col(6) = ApproxMVBB::Vector3(1,1,1);
        t.col(7) = ApproxMVBB::Vector3(0,1,1);
        mvbbTest("UnitCube",t,true,0.001,400,5,10,10);
}


TEST(MVBBTest, UnitRectangle) {
        ApproxMVBB::Matrix3Dyn t(3,4);
        t.col(0) = ApproxMVBB::Vector3(0,0,0);
        t.col(1) = ApproxMVBB::Vector3(1,0,0);
        t.col(3) = ApproxMVBB::Vector3(0,1,0);
        t.col(2) = ApproxMVBB::Vector3(1,1,0);
        mvbbTest("UnitRectangle",t,true,0.001,400,2,2,2);
}

TEST(MVBBTest, Rectangles) {
        srand(0);
        srand48(0);
        static std::mt19937 rng(TestFunctions::randomSeed);
        static std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        // Some patches
        for(int i=0;i<10;++i){
            ApproxMVBB::Matrix3Dyn t(3,4);
            t.col(0) = ApproxMVBB::Vector3(0,0,0);
            t.col(1) = ApproxMVBB::Vector3(1,0,0);
            t.col(2) = ApproxMVBB::Vector3(1,1,0);
            t.col(3) = ApproxMVBB::Vector3(0,1,0);
            applyRandomRotTrans(t,f);
            mvbbTest("Rectangles-Nr-" + std::to_string(i),t,true,0.001,400,4,4,4);
        }
}

TEST(MVBBTest, UnitPatches2D) {
        // Some patches
        static std::mt19937 rng(TestFunctions::randomSeed);
        static std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        for(int i=0;i<10;++i){
            ApproxMVBB::Matrix3Dyn t(3,500);
            for(int i=0;i<t.cols();++i){
                t.col(i) = ApproxMVBB::Vector3(uni(rng),uni(rng),0);
            }
            mvbbTest("UnitPatches2D-Nr-" + std::to_string(i),t,true,0.001,200,4,4,4);
        }
}


TEST(MVBBTest, PointsRandom100) {
        // Some patches
        static std::mt19937 rng(TestFunctions::randomSeed);
        static std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        ApproxMVBB::Matrix3Dyn t(3,100);
        t = t.unaryExpr( f );
        applyRandomRotTrans(t,f);
        mvbbTest("PointsRandom100",t);
}

TEST(MVBBTest, PointsRandom10000) {
        // Some patches
        static std::mt19937 rng(TestFunctions::randomSeed);
        static std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        ApproxMVBB::Matrix3Dyn t(3,10000);
        t = t.unaryExpr( f );
        applyRandomRotTrans(t,f);
        mvbbTest("PointsRandom10000",t);
}


TEST(MVBBTest, Bunny) {
        static std::mt19937 rng(TestFunctions::randomSeed);
        static std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };

        auto v = getPointsFromFile3D(getFileInPath("Bunny.txt"));
        Matrix3Dyn t(3,v.size());
        for(unsigned int i = 0; i<v.size(); ++i) {
            t.col(i) = v[i];
        }
        applyRandomRotTrans(t,f);
        std::cout << "Applied Transformation" << std::endl;
        mvbbTest("Bunny",t,true,0.1);
}

#ifdef ApproxMVBB_TESTS_HIGH_PERFORMANCE
TEST(MVBBTest, PointsRandom140M) {
        static std::mt19937 rng(TestFunctions::randomSeed);
        static std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        Matrix3Dyn t(3,140000000);
        t = t.unaryExpr( f );
        mvbbTest("PointsRandom140M",t,false,0.01,400,5,0,5);
}

TEST(MVBBTest, Lucy) {
        static std::mt19937 rng(TestFunctions::randomSeed);
        static std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        auto v = getPointsFromFile3D(getFileInAddPath("Lucy.txt"));
        Matrix3Dyn t(3,v.size());
        for(unsigned int i = 0; i<v.size(); ++i) {
            t.col(i) = v[i];
        }
        applyRandomRotTrans(t,f);
        mvbbTest("Lucy",t,false,100,400,5,0,5);
}
#endif

TEST(MVBBTest, PointClouds) {
        static std::mt19937 rng(TestFunctions::randomSeed);
        static std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };

        for(unsigned int k=0; k<51; k++) {
            auto v = getPointsFromFile3D(getFileInPath("PointCloud_" + std::to_string(k) +".txt"));
            Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }

            applyRandomRotTrans(t,f);
            mvbbTest("PointClouds-Nr"+std::to_string(k),t,true,0.1,400,5,3,6);
        }
}


//        {
//            Matrix3Dyn vec(3,140000000);
//            Matrix3Dyn res(3,140000000);
//            vec.setRandom();
//            Matrix33 A(3,3);
//            A.setRandom();
//            Eigen::DenseIndex size = vec.cols(), i;
//            START_TIMER(start);
//            for(i=0; i<size; ++i) {
//                vec.col(i) = A*vec.col(i);
//            }
//            //res.noalias() = A*vec;
//            STOP_TIMER_SEC(count,start);
//            std::cout << "p:" << vec.col(0)<< std::endl;
//            std::cout << "Mult Eigen Special: " << count << " sec." << std::endl;
//        }
//
//        {
//            //   generate points
//            MatrixDynDyn vec(140000000,3);
//            MatrixDynDyn A(3,3);
//            MatrixDynDyn res(140000000,3);
//            START_TIMER(start);
//            res = vec*A;
//            STOP_TIMER_SEC(count,start);
//            std::cout << "p:" << res.row(0) << std::endl;
//            std::cout << "Mult 3: " << count << " sec." << std::endl;
//
//        }


int main(int argc, char **argv) {
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}
