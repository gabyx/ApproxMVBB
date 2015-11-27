// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ComputeMVBBTests_hpp
#define ComputeMVBBTests_hpp

#include <fstream>
#include <set>


#include "TestConfig.hpp"

#include "ApproxMVBB/ComputeApproxMVBB.hpp"

#include "TestFunctions.hpp"
#include "CPUTimer.hpp"

namespace ApproxMVBB {

class ConvexHullTest {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ApproxMVBB_DEFINE_MATRIX_TYPES

    template<typename TMatrix>
    void convexHullTest(unsigned int N, const TMatrix & v, bool dumpPoints = true) {
        using namespace PointFunctions;
        using namespace TestFunctions;

        if(dumpPoints) {
            dumpPointsMatrixBinary("./ConvexHullTest" + std::to_string(N) +".bin",v);
            //dumpPointsMatrix("./ConvexHullTest"+ std::to_string(N) +".txt",v);
        }
        std::cout << "\n\nStart ConvexHull Test "+ std::to_string(N) +"" << std::endl;
        START_TIMER(start)
        ConvexHull2D c(v);
        c.compute();
        STOP_TIMER_SEC(count, start)
        std::cout << "Timings: " << count << " sec for " <<v.cols() << " points" << std::endl;
        std::cout << "End ConvexHull Test "+ std::to_string(N) +"" << std::endl;
        if(!c.verifyHull()) {
            std::cerr << "ConvexHull Test "+ std::to_string(N) +" not ok!" << std::endl;
        }



        auto ind = c.getIndices();
        std::cout << "ConvexHull Points: " << ind.size()  << std::endl;
        unsigned int j = 0;
        Matrix2Dyn qHull(2,ind.size());
        for(auto & i : ind) {
            qHull.col(j++) = v.col(i);
        }

        dumpPointsMatrixBinary("./ConvexHullTest"+ std::to_string(N) +"Out.bin",qHull);
        //dumpPointsMatrix("./ConvexHullTest"+ std::to_string(N) +"Out.txt",qHull);
    }

    void test() {
        using namespace PointFunctions;
        using namespace TestFunctions;
        {
            // generate points
            Matrix2Dyn t(2,10);
            t.setRandom();
            convexHullTest(1,t);
        }

        {
            // generate points
            Vector2List t;
            t.push_back(Vector2(0,0));
            t.push_back(Vector2(1,1));
            t.push_back(Vector2(2,2));
            t.push_back(Vector2(3,3));
            t.push_back(Vector2(-1,1));

            ApproxMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }
            convexHullTest(2,v);
        }

        {
            // generate points
            Vector2List t;
            t.push_back(Vector2(0,0));
            t.push_back(Vector2(1,1));
            t.push_back(Vector2(2,2));
            ApproxMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }
            convexHullTest(3,v);
        }

        {
            // generate points
            Vector2List t;
            t.push_back(Vector2(0,0));
            t.push_back(Vector2(1,1));
            ApproxMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }
            convexHullTest(4,v);
        }

        {
            // generate points
            Vector2List t;
            t.push_back(Vector2(0,0));
            t.push_back(Vector2(1,1));
            t.push_back(Vector2(1,-1));
            ApproxMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }

            convexHullTest(4,v);
        }

        {
            // generate points
            Vector2List t;
            t.push_back(Vector2(0,0));
            ApproxMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }

            convexHullTest(6,v);
        }

        {
            // generate points  on circle
            unsigned int max = 1000;
            ApproxMVBB::Matrix2Dyn t(2,max);
            for(unsigned int i=0; i<max; i++) {
                t.col(i) = Vector2(std::cos(0.0001/max * i) ,std::sin(0.0001/max * i) );
            }

            convexHullTest(7,t);
        }

        {
            // generate points
            auto t = getPointsFromFile2D("./PointsSimulation2DRectFail.txt");
            ApproxMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }
            convexHullTest(8,v);

        }

        {
            // generate points
            ApproxMVBB::Matrix2Dyn t(2,400);
            getPointsFromFileBinary("./PointsBadProjection.bin",t);

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
            convexHullTest(9,t);

        }

        {
            // generate points
            ApproxMVBB::Matrix2Dyn t(2,400);
            getPointsFromFileBinary("./PointsBadProjection.bin",t);

            convexHullTest(10,t);

        }
        {
            // generate points
            ApproxMVBB::Matrix2Dyn t(2,400);
            getPointsFromFileBinary("./PointsBadProjection2.bin",t);

            convexHullTest(11,t);

        }
        {
            // generate points
            ApproxMVBB::Matrix2Dyn t(2,400);
            getPointsFromFileBinary("./PointsBadProjection3.bin",t);

            convexHullTest(12,t);

        }

        {
            // generate points
            ApproxMVBB::Matrix2Dyn t(2,16);
            t.setZero();
            getPointsFromFileBinary("./PointsBadProjection4.bin",t);
            convexHullTest(13,t);

        }

        {
            // generate points
            ApproxMVBB::Matrix2Dyn t(2,5);
            t.setZero();
            getPointsFromFileBinary("./PointsBadProjection5.bin",t);
            convexHullTest(14,t);

        }

        {
            // generate points
            ApproxMVBB::Matrix2Dyn t(2,100);
            t.setZero();
            getPointsFromFileBinary("./PointsBadProjection6.bin",t);
            convexHullTest(15,t);

        }
#ifdef ApproxMVBB_TESTS_HIGH_PERFORMANCE
        {
            // generate points
            ApproxMVBB::Matrix2Dyn t(2,14000000);
            t.setRandom();

            convexHullTest(14,t,false);

        }
#endif
    }
};


void convexHullTest() {
    srand(0);
    ConvexHullTest t;
    t.test();
}



class MinAreaRectangleTest {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ApproxMVBB_DEFINE_MATRIX_TYPES


    template<typename TMatrix>
    void minRectTest(unsigned int N, const TMatrix & v) {
        using namespace PointFunctions;
        using namespace TestFunctions;

        dumpPointsMatrixBinary("./MinAreaRectangleTest" + std::to_string(N) +".bin",v);
        dumpPointsMatrix("./MinAreaRectangleTest"+ std::to_string(N) +".txt",v);

        std::cout << "\n\nStart MinAreaRectangle Test "+ std::to_string(N) +"" << std::endl;
        START_TIMER(start)

        MinAreaRectangle c(v);
        c.compute();

        STOP_TIMER_SEC(count, start)
        std::cout << "Timings: " << count << " sec for " <<v.cols() << " points" << std::endl;
        std::cout << "End MinAreaRectangle Test "+ std::to_string(N) +"" << std::endl;
        auto rect = c.getMinRectangle();

        Matrix2Dyn p(2,7);
        p.col(0) =  rect.m_p;
        p.col(1) =  rect.m_p + rect.m_u*rect.m_uL ;
        p.col(2) =  rect.m_p + rect.m_u*rect.m_uL + rect.m_v*rect.m_vL ;
        p.col(3) =  rect.m_p + rect.m_v*rect.m_vL ;
        p.col(4) =  rect.m_p;
        p.col(5) =  rect.m_u;
        p.col(6) =  rect.m_v;

        dumpPointsMatrixBinary("./MinAreaRectangleTest"+ std::to_string(N) +"Out.bin",p);
        dumpPointsMatrix("./MinAreaRectangleTest"+ std::to_string(N) +"Out.txt",p);
    }

    void test() {
        using namespace PointFunctions;
        using namespace TestFunctions;
        {
            // generate points
            Matrix2Dyn t(2,10);
            t.setRandom();
            minRectTest(1,t);
        }

        {
            // generate points
            Vector2List v;
            v.push_back(Vector2(0,0));
            v.push_back(Vector2(1,1));
            v.push_back(Vector2(2,2));
            v.push_back(Vector2(3,3));
            v.push_back(Vector2(-1,1));
            Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(2,t);
        }

        {
            // generate points
            Vector2List v;
            v.push_back(Vector2(0,0));
            v.push_back(Vector2(1,1));
            v.push_back(Vector2(2,2));
            Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(3,t);
        }

        {
            // generate points
            Vector2List v;
            v.push_back(Vector2(0,0));
            v.push_back(Vector2(1,1));
            ApproxMVBB::Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(4,t);
        }

        {
            // generate points
            Vector2List v;
            v.push_back(Vector2(0,0));
            v.push_back(Vector2(1,1));
            v.push_back(Vector2(1,-1));
            Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(5,t);
        }

        {
            // generate points
            Vector2List v;
            v.push_back(Vector2(0,0));
            v.push_back(Vector2(1,1));
            v.push_back(Vector2(1,1+1e-13));
            v.push_back(Vector2(2,2));
            Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(6,t);
        }
//
//        {
//            // generate points  on circle
//            unsigned int max = 100000;
//            Vector2List v(max);
//            for(unsigned int i=0; i<max; i++) {
//                v[i] = Vector2(std::cos(0.0001/max * i) ,std::sin(0.0001/max * i) );
//            }
//            ApproxMVBB::Matrix2Dyn t(2,v.size());
//            for(unsigned int i = 0; i<v.size(); ++i) {
//                t.col(i) = v[i];
//            }
//            minRectTest(7,t);
//        }


        {
            // generate points
            Vector2List v;
            v.push_back(Vector2(1,0));

            Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(8,t);
        }

        {
            // generate points
            Vector2List v;

            Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(9,t);
        }

        {

            // generate points
            Vector2List v;
            v.push_back(Vector2(0,0));
            v.push_back(Vector2(1,0));
            v.push_back(Vector2(1,1));
            v.push_back(Vector2(0,1));
            Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(10,t);
        }



        {
            // generate points
            auto v = getPointsFromFile2D("./PointsSimulation2DRectFail.txt");
            Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(11,t);
        }

#ifdef ApproxMVBB_TESTS_HIGH_PERFORMANCE
        {
            // generate points
            Matrix2Dyn t(2,10000000);
            t.setRandom();
            minRectTest(12,t);
        }
#endif


        {
            // generate points
            Matrix2Dyn t(2,400);
            getPointsFromFileBinary("./PointsBadProjection.bin",t);
            minRectTest(13,t);
        }

        {
            // generate points
            Matrix2Dyn t(2,400);
            getPointsFromFileBinary("./PointsBadProjection2.bin",t);
            minRectTest(14,t);
        }

        {
            // generate points
            Matrix2Dyn t(2,400);
            getPointsFromFileBinary("./PointsBadProjection3.bin",t);
            minRectTest(15,t);
        }

    }
};


void minAreaBoxTest() {
    srand(0);
    MinAreaRectangleTest t;
    t.test();
}



class DiameterTest {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ApproxMVBB_DEFINE_MATRIX_TYPES

    template<typename TMatrix>
    void diameterTest(unsigned int N, const TMatrix & v, bool dump = true,
                      unsigned int optLoops = 10, PREC epsilon = 0.001, unsigned int samplePoints = 400) {
        using namespace PointFunctions;
        using namespace TestFunctions;

        if(dump) {
            dumpPointsMatrixBinary("./DiameterTest" + std::to_string(N) +".bin",v);
            dumpPointsMatrix("./DiameterTest" + std::to_string(N) +".txt",v);
        }

        std::cout << "\n\nStart approximateMVBBDiam Test "+ std::to_string(N) +"" << std::endl;
        START_TIMER(start)
        auto oobb = ApproxMVBB::approximateMVBBDiam(v,epsilon,optLoops);
        STOP_TIMER_SEC(count, start)
        std::cout << "Timings: " << count << " sec for " <<v.cols() << " points" << std::endl;
        std::cout << "End approximateMVBBDiam Test "+ std::to_string(N) << std::endl;

        oobb.expand(1e-10);
        if(!checkPointsInOOBB(v,oobb)) {
            std::cout << "WARNING: Not all points in OOBB.expand(1e-10)" << std::endl;
        } else {
            std::cout << "All points in OOBB!" << std::endl;
        }

        std::cout << "Start Sampling Test "+ std::to_string(N) +"" << std::endl;
        Matrix3Dyn sampled;
        START_TIMER(start2)
        ApproxMVBB::samplePointsGrid(sampled,v,samplePoints,oobb);
        STOP_TIMER_SEC(count2, start2)
        std::cout << "Timings: " << count2 << " sec for " <<sampled.cols() << " points" << std::endl;
        std::cout << "End Sampling Test "+ std::to_string(N) << std::endl;

        //oobb = ApproxMVBB::optimizeMVBB(sampled,oobb,2);

        dumpOOBB("./DiameterTest"+ std::to_string(N) +"Out.txt", oobb);

        dumpPointsMatrixBinary("./DiameterTest"+ std::to_string(N) +"Out2.bin",sampled);
        dumpPointsMatrix("./DiameterTest" + std::to_string(N) +"Out2.txt",sampled);


        ApproxMVBB_WARNINGMSG(oobb.volume() > 1e-6,"Volume small: " << oobb.volume() << std::endl)


    }


    void test() {
        using namespace PointFunctions;
        using namespace TestFunctions;

        {
            // generate points
            Matrix3Dyn t(3,500);
            t.setRandom();
            diameterTest(1,t);
        }

        {
            // generate points
            Matrix3Dyn t(3,10000);
            t.setRandom();
            diameterTest(2,t);
        }

        {
            // generate points
            auto v = getPointsFromFile3D("./PointsSimulation.txt");

            Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            diameterTest(3,t);
        }

        {
            // generate points
            auto v = getPointsFromFile3D("./PointsSimulationFailMVBB.txt");
            Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            diameterTest(4,t,true,10);
        }

        {
            // generate points
            auto v = getPointsFromFile3D("./Bunny.txt");

            Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            diameterTest(5,t,false,10,1);
        }

        {
            // generate points
            Matrix3Dyn t(3,140000000);
            t.setRandom();
            diameterTest(7,t,false,0,0.01);
        }

#ifdef ApproxMVBB_TESTS_HIGH_PERFORMANCE

        {
            // generate points
            auto v = getPointsFromFile3D("./Lucy.txt");

            Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            diameterTest(6,t,false,3,100);
        }

        {
            // generate points
            Matrix3Dyn t(3,140000000);
            t.setRandom();
            diameterTest(7,t,false,0,0.01);
        }


#endif


        //  Tests 8 - 59
        //for(unsigned int k=0;k<5;k++){
        for(unsigned int k=0; k<51; k++) {

            // generate points
            auto v = getPointsFromFile3D("./PointCloud_" + std::to_string(k) +".txt");

            Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            PointFunctions::applyRandomRotTrans(t);
            diameterTest(/*k*51 +*/k+8,t,true,4,0.1);
        }
        //}

        {
            // generate points
            Matrix3Dyn t(3,3);
            t.setRandom();
            diameterTest(60,t,true,10,0.001,2);
        }

    }
};


void diameterTest() {
    srand(time(NULL));
    ///srand(0);
    DiameterTest t;
    t.test();
}



class MVBBTests {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ApproxMVBB_DEFINE_MATRIX_TYPES

    template<typename TMatrix>
    void mvbbTest(unsigned int N, const TMatrix & v,
                  bool dumpPoints = true,
                  PREC eps = 0.001,
                  unsigned int nPoints = 400,
                  unsigned int gridSize = 5,
                  unsigned int mvbbDiamOptLoops = 2,
                  unsigned int gridSearchOptLoops = 10
                 ) {
        using namespace PointFunctions;
        using namespace TestFunctions;

        if(dumpPoints) {
            dumpPointsMatrix("./MVBBTest" + std::to_string(N) +".txt",v);
            dumpPointsMatrixBinary("./MVBBTest" + std::to_string(N) +".bin",v);
        }

        std::cout << "\n\nStart MVBBTest Test "+ std::to_string(N) +"" << std::endl;
        START_TIMER(start)
        auto oobb = ApproxMVBB::approximateMVBB(v,eps,nPoints,gridSize,mvbbDiamOptLoops,gridSearchOptLoops);
        STOP_TIMER_SEC(count, start)
        std::cout << "Timings: " << count << " sec for " <<v.cols() << " points" << std::endl;
        std::cout << "End MVBBTest "+ std::to_string(N) << std::endl;

        ApproxMVBB_WARNINGMSG(oobb.volume() > 1e-6,"Volume small: " << oobb.volume() << std::endl)

        oobb.expandToMinExtentRelative(0.1);

        // Make all points inside OOBB!
        Matrix33 A_KI = oobb.m_q_KI.matrix().transpose(); // faster to store the transformation matrix first
        auto size = v.cols();
        for( unsigned int i=0;  i<size; ++i ) {
            oobb.unite(A_KI*v.col(i));
        }

        dumpOOBB("./MVBBTest"+ std::to_string(N) +"Out.txt", oobb);


    }

    void test() {
        using namespace PointFunctions;
        using namespace TestFunctions;

         {
            // generate points
            Matrix3Dyn t(3,1);
            t.setRandom();
            mvbbTest(60,t,true,0.001,1,5,10,10);
        }

        {
            // generate points
            Matrix3Dyn t(3,2);
            t.setRandom();
            mvbbTest(61,t,true,0.001,2,5,10,10);
        }

        {
            // generate points
            Matrix3Dyn t(3,3);
            t.setRandom();
            mvbbTest(62,t,true,0.001,3,5,10,10);
        }

        {
            // generate points
            auto v = generatePoints3D(100);

            Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            mvbbTest(1,t);
        }

        {
            // generate points

            auto v = generatePoints3D(10000);

            Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            mvbbTest(2,t);


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

        {
            // generate points
            auto v = getPointsFromFile3D("./Bunny.txt");

            Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            mvbbTest(3,t,true,0.1);
        }




#ifdef ApproxMVBB_TESTS_HIGH_PERFORMANCE

        {
            // generate points
            auto v = getPointsFromFile3D("./Lucy.txt");
            Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }

            applyRandomRotTrans(t);
            mvbbTest(4,t,false,100,400,5,0,5);
        }

         {
            // generate points
            Matrix3Dyn t(3,140000000);
            t.setRandom();
            mvbbTest(4,t,false,0.01,400,5,0,5);
        }
#endif

         // Tests 9 - 59
        //for(unsigned int k=0;k<5;k++){
            for(unsigned int k=0;k<51;k++){

                // generate points
                auto v = getPointsFromFile3D("./PointCloud_" + std::to_string(k) +".txt");
                ApproxMVBB::Matrix3Dyn t(3,v.size());
                for(unsigned int i = 0; i<v.size(); ++i) {
                    t.col(i) = v[i];
                }

                PointFunctions::applyRandomRotTrans(t);
                mvbbTest(/*k*51+*/k + 4,t,true,0.1,400,5,3,6);
            }
        //}

    }

};


void mvbbTest() {
    srand(0);
    MVBBTests t;
    t.test();
}

}

#endif
