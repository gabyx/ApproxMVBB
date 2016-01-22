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

namespace DiameterOOBBTest {

    std::string genTestName( std::string name){
        return "DiameterOOBBTest-" + name;
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
    void diameterTest(std::string name, const TMatrix & v, bool dump = true,
                      unsigned int optLoops = 10, PREC epsilon = 0.001, unsigned int samplePoints = 400,
                      bool checkVolume = true
                      ) {
        using namespace PointFunctions;
        using namespace TestFunctions;

        std::cout << " Set random hash for std : " << setRandomSeedStd(name) << std::endl;

        if(dump) {
            dumpPointsMatrixBinary( getPointsDumpPath(name,".bin") ,v);
            dumpPointsMatrix( getPointsDumpPath(name,".txt"),v);
        }

        std::cout << "\n\nStart approximateMVBBDiam Test "+ name +"" << std::endl;
        START_TIMER(start)
        auto oobb = ApproxMVBB::approximateMVBBDiam(v,epsilon,optLoops);
        STOP_TIMER_SEC(count, start)
        std::cout << "Timings: " << count << " sec for " <<v.cols() << " points" << std::endl;
        std::cout << "End approximateMVBBDiam Test "+ name << std::endl;

        oobb.expand(1e-10);
        if(!checkPointsInOOBB(v,oobb)) {
            std::cout << "WARNING: Not all points in OOBB.expand(1e-10)" << std::endl;
        } else {
            std::cout << "All points in OOBB!" << std::endl;
        }

        std::cout << "Start Sampling Test "+ name +"" << std::endl;
        Matrix3Dyn sampled;
        std::mt19937 rng(TestFunctions::randomSeed); // always generate the same indices in samplePointsGrid
        START_TIMER(start2)
        ApproxMVBB::samplePointsGrid(sampled,v,samplePoints,oobb, rng);
        STOP_TIMER_SEC(count2, start2)
        std::cout << "Timings: " << count2 << " sec for " <<sampled.cols() << " points" << std::endl;
        std::cout << "End Sampling Test "+ name << std::endl;

        //oobb = ApproxMVBB::optimizeMVBB(sampled,oobb,2);

        dumpOOBB(getFileOutPath(name,".txt"), oobb);
        dumpPointsMatrixBinary(getFileOutPath(name,"2.bin"),sampled);
        //dumpPointsMatrix(getFileOutPath(name,"2.txt"),sampled);

        // Check OOBB
        try{

            readOOBBAndCheck( oobb, getFileValidationPath(name,".txt") );

            // Check Sampled points
            Matrix3Dyn valid = sampled;
            valid.setConstant(std::numeric_limits<PREC>::signaling_NaN());
            readPointsMatrixBinary( getFileValidationPath(name,"2.bin") , valid);
            EXPECT_TRUE( assertNearArrays(sampled,valid));

            if(checkVolume){
                ASSERT_GT(oobb.volume() , 1e-6)  << "Volume too small: " << oobb.volume() << std::endl;
            }
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
using namespace ApproxMVBB::DiameterOOBBTest;

TEST(DiameterOOBBTest, PointsRandom500) {
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        // generate points
        Matrix3Dyn t(3,500);
        t = t.unaryExpr( f );
        diameterTest("PointsRandom500",t);
}

TEST(DiameterOOBBTest, PointsRandom10000) {
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        // generate points
        Matrix3Dyn t(3,10000);
        t = t.unaryExpr( f );
        diameterTest("PointsRandom10000",t);
}

TEST(DiameterOOBBTest, UnitCube) {
        Matrix3Dyn t(3,8);
        t.col(0) = ApproxMVBB::Vector3(0,0,0);
        t.col(1) = ApproxMVBB::Vector3(1,0,0);
        t.col(2) = ApproxMVBB::Vector3(1,1,0);
        t.col(3) = ApproxMVBB::Vector3(0,1,0);
        t.col(4) = ApproxMVBB::Vector3(0,0,1);
        t.col(5) = ApproxMVBB::Vector3(1,0,1);
        t.col(6) = ApproxMVBB::Vector3(1,1,1);
        t.col(7) = ApproxMVBB::Vector3(0,1,1);

        diameterTest("UnitCube",t,true,1,0.001,4);
}

TEST(DiameterOOBBTest, PointsSimulation) {
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };

        auto v = getPointsFromFile3D(getFileInPath("PointsSimulation.txt"));

        Matrix3Dyn t(3,v.size());
        for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
        }
        applyRandomRotTrans(t,f);
        std::cout << "Applied Transformation" << std::endl;
        diameterTest("PointsSimulation",t);
}

TEST(DiameterOOBBTest, PointsSimulationFailMVBB) {
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };

        auto v = getPointsFromFile3D(getFileInPath("PointsSimulationFailMVBB.txt"));
        Matrix3Dyn t(3,v.size());
        for(unsigned int i = 0; i<v.size(); ++i) {
        t.col(i) = v[i];
        }
        applyRandomRotTrans(t,f);
        std::cout << "Applied Transformation" << std::endl;
        diameterTest("PointsSimulationFailMVBB",t,true,10);
}

TEST(DiameterOOBBTest, Bunny) {
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };

        auto v = getPointsFromFile3D(getFileInPath("Bunny.txt"));
        Matrix3Dyn t(3,v.size());
        for(unsigned int i = 0; i<v.size(); ++i) {
        t.col(i) = v[i];
        }
        applyRandomRotTrans(t,f);
        std::cout << "Applied Transformation" << std::endl;
        diameterTest("Bunny",t,false,10,1);
}

#ifdef ApproxMVBB_TESTS_HIGH_PERFORMANCE
TEST(DiameterOOBBTest, PointsRandom14M) {
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        Matrix3Dyn t(3,140000000);
        t = t.unaryExpr( f );
        diameterTest("PointsRandom14M",t,false,0,0.01);
}


TEST(DiameterOOBBTest, Lucy) {
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        auto v = getPointsFromFile3D(getFileInAddPath("Lucy.txt"));
        Matrix3Dyn t(3,v.size());
        for(unsigned int i = 0; i<v.size(); ++i) {
            t.col(i) = v[i];
        }
        applyRandomRotTrans(t,f);
        diameterTest("Lucy",t,false,3,100);
}
#endif

TEST(DiameterOOBBTest, Plane) {
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };

        Matrix3Dyn t(3,3);
        t = t.unaryExpr( f );
        diameterTest("Plane",t,true,10,0.001,2,false);
}

TEST(DiameterOOBBTest, PointClouds) {
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };

        for(unsigned int k=0; k<51; k++) {
            auto v = getPointsFromFile3D(getFileInPath("PointCloud_" + std::to_string(k) +".txt"));
            Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }

            applyRandomRotTrans(t,f);
            diameterTest("PointClouds-Nr"+std::to_string(k),t,true,4,0.1);
        }
}

TEST(DiameterOOBBTest, UnitPatches2D) {
        // Some patches
        std::mt19937 rng(TestFunctions::randomSeed);
        std::uniform_real_distribution<PREC> uni(0.0,1.0);
        auto f = [&](PREC) { return uni(rng); };
        for(int i=0;i<10;++i){
            ApproxMVBB::Matrix3Dyn t(3,500);
            for(int i=0;i<t.cols();++i){
                t.col(i) = ApproxMVBB::Vector3(uni(rng),uni(rng),0);
            }
            diameterTest("UnitPatches2D-Nr-" + std::to_string(i),t,true,10,0.0,400,false);
        }
}

//TEST(DISABLED_DiameterOOBBTest, Plane) {
//
//        Matrix3Dyn t(3,3);
//        t.setConstant(std::numeric_limits<PREC>::signaling_NaN());
//        diameterTest("Plane",t,true,10,0.001,2);
//}



int main(int argc, char **argv) {
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}
