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

namespace DiameterTest {

    std::string genTestName( std::string name){
        return "DiameterTest-" + name;
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
        return ApproxMVBB_TESTS_OUTPUT_FILES_DIR "/" + genTestName(name)+"-"+getPrecAbrev()+"Points-Out"+suffix;
    }

    std::string getFileOutPath(std::string name, std::string suffix=".bin"){
        return ApproxMVBB_TESTS_OUTPUT_FILES_DIR "/" + genTestName(name) +"-"+getPrecAbrev()+"-"+"Out"+suffix;
    }
    std::string getFileValidationPath(std::string name, std::string suffix=".bin"){
        return ApproxMVBB_TESTS_VALIDATION_FILES_DIR "/" + getPrecAbrev()+"/"+genTestName(name) +"-"+getPrecAbrev()+"-"+"Out"+suffix;
    }

    template<typename TMatrix>
    void diameterTest(std::string name, const TMatrix & v, PREC epsilon = 0.001) {

        using namespace PointFunctions;
        using namespace TestFunctions;

        TMatrix in = v;
        std::cout << "\n\nStart estimateDiam test "+ name +"" << std::endl;
        START_TIMER(start)
        auto pp = estimateDiameter< 3 >(v, epsilon);
        STOP_TIMER_SEC(count, start)
        std::cout << "Timings: " << count << " sec for " <<v.cols() << " points" << std::endl;
        std::cout << "End estimateDiam test "+ name << std::endl;



        Matrix3Dyn diam(3,2);
        diam.col(0) = pp.first;
        diam.col(1) = pp.second;
        dumpPointsMatrixBinary( getFileOutPath(name,".bin" ),diam); // dumps diameter
        dumpPointsMatrixBinary( getFileOutPath(name,"2.bin") ,v);   // dump generated points

        // CHECK
        try{
            Matrix3Dyn diamV(3,2);
            readPointsMatrixBinary( getFileValidationPath(name,".bin"), diamV );
            Matrix3Dyn inputPointsV;
            readPointsMatrixBinary( getFileValidationPath(name,"2.bin"), inputPointsV );

            EXPECT_TRUE( (in.array() == v.array()).all() ) << " estimate diameter changed the point cloud! should not happed!";

            EXPECT_TRUE( assertNearArrayColsRows(diam,diamV)) << "Diameter valid: " << std::endl << diamV
            << std::endl << "Diameter computed: " << std::endl << diam << std::endl;

            EXPECT_TRUE( assertNearArray(v,inputPointsV) ) ;
        }
        catch( ApproxMVBB::Exception & e){
            ASSERT_TRUE(false) << "Exception in checking inside test!: "  << e.what() << std::endl;
        }
    }

};
};

using namespace ApproxMVBB;
using namespace TestFunctions;
using namespace PointFunctions;
using namespace ApproxMVBB::DiameterTest;


MY_TEST(DiameterTest, PointsRandom3) {
MY_TEST_RANDOM_STUFF(PointsRandom3)
        // generate points
        Matrix3Dyn t(3,3);
        t = t.unaryExpr( f );
        diameterTest(testName,t);
}


MY_TEST(DiameterTest, PointsRandom500) {
MY_TEST_RANDOM_STUFF(PointsRandom500)
        // generate points
        Matrix3Dyn t(3,500);
        t = t.unaryExpr( f );
        diameterTest(testName,t);
}


MY_TEST(DiameterTest, PointsRandom10000) {
MY_TEST_RANDOM_STUFF(PointsRandom10000)
        // generate points
        Matrix3Dyn t(3,10000);
        t = t.unaryExpr( f );
        diameterTest(testName,t);
}

MY_TEST(DiameterTest, UnitCube) {
MY_TEST_RANDOM_STUFF(UnitCube)
        Matrix3Dyn t(3,8);
        t.col(0) = ApproxMVBB::Vector3(0,0,0);
        t.col(1) = ApproxMVBB::Vector3(1,0,0);
        t.col(2) = ApproxMVBB::Vector3(1,1,0);
        t.col(3) = ApproxMVBB::Vector3(0,1,0);
        t.col(4) = ApproxMVBB::Vector3(0,0,1);
        t.col(5) = ApproxMVBB::Vector3(1,0,1);
        t.col(6) = ApproxMVBB::Vector3(1,1,1);
        t.col(7) = ApproxMVBB::Vector3(0,1,1);

        diameterTest(testName,t);
}

//MY_TEST(DiameterTest, PointsSimulation) {
//MY_TEST_RANDOM_STUFF(PointsSimulation)
//        auto v = getPointsFromFile3D(getFileInPath("PointsSimulation.txt"));
//
//        Matrix3Dyn t(3,v.size());
//        for(unsigned int i = 0; i<v.size(); ++i) {
//                t.col(i) = v[i];
//        }
//        applyRandomRotTrans(t,f);
//        std::cout << "Applied Transformation" << std::endl;
//        diameterTest(testName,t);
//}
//
//MY_TEST(DiameterTest, PointsSimulationFailMVBB) {
//MY_TEST_RANDOM_STUFF(PointsSimulationFailMVBB)
//        auto v = getPointsFromFile3D(getFileInPath("PointsSimulationFailMVBB.txt"));
//        Matrix3Dyn t(3,v.size());
//        for(unsigned int i = 0; i<v.size(); ++i) {
//        t.col(i) = v[i];
//        }
//        applyRandomRotTrans(t,f);
//        std::cout << "Applied Transformation" << std::endl;
//        diameterTest(testName,t);
//}
//
//MY_TEST(DiameterTest, Bunny) {
//MY_TEST_RANDOM_STUFF(Bunny)
//        auto v = getPointsFromFile3D(getFileInPath("Bunny.txt"));
//        Matrix3Dyn t(3,v.size());
//        for(unsigned int i = 0; i<v.size(); ++i) {
//                t.col(i) = v[i];
//        }
//        applyRandomRotTrans(t,f);
//        std::cout << "Applied Transformation" << std::endl;
//        diameterTest(testName,t);
//}
//
//#ifdef ApproxMVBB_TESTS_HIGH_PERFORMANCE
//MY_TEST(DiameterTest, PointsRandom14M) {
//MY_TEST_RANDOM_STUFF(PointsRandom14M)
//        Matrix3Dyn t(3,140000000);
//        t = t.unaryExpr( f );
//        diameterTest(testName,t);
//}
//
//
//MY_TEST(DiameterTest, Lucy) {
//MY_TEST_RANDOM_STUFF(Lucy)
//        auto v = getPointsFromFile3D(getFileInAddPath("Lucy.txt"));
//        Matrix3Dyn t(3,v.size());
//        for(unsigned int i = 0; i<v.size(); ++i) {
//            t.col(i) = v[i];
//        }
//        applyRandomRotTrans(t,f);
//        diameterTest(testName,t);
//}
//#endif

MY_TEST(DiameterTest, Plane) {
MY_TEST_RANDOM_STUFF(Plane)
        Matrix3Dyn t(3,3);
        t = t.unaryExpr( f );
        diameterTest(testName,t,true);
}

//MY_TEST(DiameterTest, PointClouds) {
//MY_TEST_RANDOM_STUFF(PointClouds)
//
//        for(unsigned int k=0; k<51; k++) {
//            auto v = getPointsFromFile3D(getFileInPath("PointCloud_" + std::to_string(k) +".txt"));
//            Matrix3Dyn t(3,v.size());
//            for(unsigned int i = 0; i<v.size(); ++i) {
//                t.col(i) = v[i];
//            }
//
//            applyRandomRotTrans(t,f);
//            diameterTest("PointClouds-Nr"+std::to_string(k),t);
//        }
//}

MY_TEST(DiameterTest, UnitPatches2D) {
MY_TEST_RANDOM_STUFF(UnitPatches2D)
        for(int i=0;i<10;++i){
            ApproxMVBB::Matrix3Dyn t(3,500);
            t = t.unaryExpr(f);
            diameterTest("UnitPatches2D-Nr-" + std::to_string(i),t);
        }
}

//MY_TEST(DISABLED_DiameterTest, Plane) {
//MY_TEST_RANDOM_STUFF(Plane)
//
//        Matrix3Dyn t(3,3);
//        t.setConstant(std::numeric_limits<PREC>::signaling_NaN());
//        diameterTest(testName,t);
//}



int main(int argc, char **argv) {
        testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
}
