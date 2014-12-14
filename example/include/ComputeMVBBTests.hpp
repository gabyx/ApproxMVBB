// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef ComputeMVBBTests_hpp
#define ComputeMVBBTests_hpp

#include <fstream>

#include "MVBB/ComputeMVBB.hpp"
#include "MVBB/Common/CPUTimer.hpp"

namespace TestFunctions{

    ApproxMVBB_DEFINE_MATRIX_TYPES
    DEFINE_POINTS_CONFIG_TYPES


    template<typename Derived>
    void dumpPointsMatrix(std::string filePath, const MatrixBase<Derived> & v) {

        std::ofstream l;
        l.open(filePath.c_str());
        if(!l.good()){
            ERRORMSG("Could not open file: " << filePath << std::endl)
        }

        for(unsigned int i=0; i<v.cols(); i++) {
            l << v.col(i).transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        }
    }

    template<typename Container>
    void dumpPoints(std::string filePath, Container & c) {

        std::ofstream l;
        l.open(filePath.c_str());
        if(!l.good()){
            ERRORMSG("Could not open file: " << filePath << std::endl)
        }

        for(auto & v: c) {
            l << v.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        }
    }

    void dumpOOBB(std::string filePath, OOBB & oobb) {

        std::ofstream l;
        l.open(filePath.c_str());
        if(!l.good()){
            ERRORMSG("Could not open file: " << filePath << std::endl)
        }

        l << oobb.m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        l << oobb.m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        l << oobb.m_q_KI.matrix().format(MyMatrixIOFormat::SpaceSep) << std::endl;
    }

    PointFunctions::Vector3List getPointsFromFile3D(std::string filePath) {

        std::ifstream file;            //creates stream myFile
        file.open(filePath.c_str());  //opens .txt file

        if (!file.is_open()) { // check file is open, quit if not
            ERRORMSG("Could not open file: " << filePath)
        }

        PREC a,b,c;
        Vector3List v;
        while(file.good()) {
            file >> a>>b>>c;
            v.emplace_back(a,b,c);
        }
        file.close();
        return v;
    }

    Vector2List getPointsFromFile2D(std::string filePath) {

        std::ifstream file;            //creates stream myFile
        file.open(filePath.c_str());  //opens .txt file

        if (!file.is_open()) { // check file is open, quit if not
            ERRORMSG("Could not open file: " << filePath)
        }
        PREC a,b;
        Vector2List v;
        while(file.good()) {
            file >> a>>b;
            v.emplace_back(a,b);
        }
        file.close();
        return v;
    }

    Vector2List generatePoints2D(unsigned int n=4) {
        Vector2List v;
        for(unsigned int i=0; i<n; i++) {
            v.push_back( Vector2::Random() );
        }
        return v;
    }

    Vector3List generatePoints3D(unsigned int n=4) {
        Vector3List v;
        for(unsigned int i=0; i<n; i++) {
            v.push_back( Vector3::Random() );
        }
        return v;
    }


    template<typename Derived>
    bool checkPointsInOOBB(const MatrixBase<Derived> & points, OOBB oobb){

        Matrix33 A_KI  = oobb.m_q_KI.matrix();
        A_KI.transposeInPlace();

        Vector3 p;
        bool allInside = true;
        auto size = points.cols();
        decltype(size) i = 0;
        while(i<size && allInside){
                p = A_KI * points.col(i);
                allInside &= (   p(0) >= oobb.m_minPoint(0) && p(0) <= oobb.m_maxPoint(0) &&
                                 p(1) >= oobb.m_minPoint(1) && p(1) <= oobb.m_maxPoint(1) &&
                                 p(2) >= oobb.m_minPoint(2) && p(2) <= oobb.m_maxPoint(2));
                ++i;
        }
        return allInside;
    }

};


class ConvexHullTest {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ApproxMVBB_DEFINE_MATRIX_TYPES

    template<typename TMatrix>
    void convexHullTest(unsigned int N, const TMatrix & v) {
        using namespace PointFunctions;
        using namespace TestFunctions;

        dumpPointsMatrix("./ConvexHullTest" + std::to_string(N) +".txt",v);

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
        unsigned int j = 0;
        ComputeMVBB::Matrix2Dyn qHull(2,ind.size());
        for(auto & i : ind) {
            qHull.col(j++) = v.col(i);
        }

        dumpPointsMatrix("./ConvexHullTest"+ std::to_string(N) +"Out.txt",qHull);

    }

    void test() {
        using namespace PointFunctions;
        using namespace TestFunctions;
        {
            std::cerr << "Convex Hull Test 1 " << std::endl;
            // generate points
            ComputeMVBB::Matrix2Dyn t(2,10);
            t.setRandom();
            convexHullTest(1,t);
        }

        {
            std::cerr << "Convex Hull Test 2 " << std::endl;
            // generate points
            Vector2List t;
            t.push_back(Vector2(0,0));
            t.push_back(Vector2(1,1));
            t.push_back(Vector2(2,2));
            t.push_back(Vector2(3,3));
            t.push_back(Vector2(-1,1));

            ComputeMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }
            convexHullTest(2,v);
        }

        {
            std::cerr << "Convex Hull Test 3 " << std::endl;
            // generate points
            Vector2List t;
            t.push_back(Vector2(0,0));
            t.push_back(Vector2(1,1));
            t.push_back(Vector2(2,2));
            ComputeMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }
            convexHullTest(3,v);
        }

        {
            std::cerr << "Convex Hull Test 4 " << std::endl;
            // generate points
            Vector2List t;
            t.push_back(Vector2(0,0));
            t.push_back(Vector2(1,1));
            ComputeMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }
            convexHullTest(4,v);
        }

        {
            std::cerr << "Convex Hull Test 5 " << std::endl;
            // generate points
            Vector2List t;
            t.push_back(Vector2(0,0));
            t.push_back(Vector2(1,1));
            t.push_back(Vector2(1,-1));
            ComputeMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }

            convexHullTest(4,v);
        }

        {
            std::cerr << "Convex Hull Test 6 " << std::endl;
            // generate points
            Vector2List t;
            t.push_back(Vector2(0,0));
            ComputeMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }

            convexHullTest(6,v);
        }

        {
            // generate points  on circle
            unsigned int max = 10000;
            ComputeMVBB::Matrix2Dyn t(2,max);
            for(unsigned int i=0; i<max; i++) {
                t.col(i) = Vector2(std::cos(0.0001/max * i) ,std::sin(0.0001/max * i) );
            }

            convexHullTest(7,t);
        }

        {
            // generate points
            auto t = getPointsFromFile2D("./PointsSimulation2DRectFail.txt");
            ComputeMVBB::Matrix2Dyn v(2,t.size());
            for(unsigned int i = 0; i<t.size(); ++i) {
                v.col(i) = t[i];
            }
            convexHullTest(8,v);

        }


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

        dumpPointsMatrix("./MinAreaRectangleTest" + std::to_string(N) +".txt",v);

        std::cout << "\n\nStart MinAreaRectangle Test "+ std::to_string(N) +"" << std::endl;
        START_TIMER(start)

        MinAreaRectangle c(v);
        c.compute();

        STOP_TIMER_SEC(count, start)
        std::cout << "Timings: " << count << " sec for " <<v.cols() << " points" << std::endl;
        std::cout << "End MinAreaRectangle Test "+ std::to_string(N) +"" << std::endl;
        auto rect = c.getMinRectangle();

        Vector2List p;
        p.push_back( rect.m_p);
        p.push_back( rect.m_p + rect.m_u );
        p.push_back( rect.m_p + rect.m_u + rect.m_v );
        p.push_back( rect.m_p + rect.m_v );
        p.push_back( rect.m_p);

        dumpPoints("./MinAreaRectangleTest"+ std::to_string(N) +"Out.txt",p);

    }

    void test() {
        using namespace PointFunctions;
        using namespace TestFunctions;
        {
            // generate points
            ComputeMVBB::Matrix2Dyn t(2,10);
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
            ComputeMVBB::Matrix2Dyn t(2,v.size());
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
            ComputeMVBB::Matrix2Dyn t(2,v.size());
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
            ComputeMVBB::Matrix2Dyn t(2,v.size());
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
            ComputeMVBB::Matrix2Dyn t(2,v.size());
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
            ComputeMVBB::Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(6,t);
        }

        {
            // generate points  on circle
            unsigned int max = 100000;
            Vector2List v(max);
            for(unsigned int i=0; i<max; i++) {
                v[i] = Vector2(std::cos(0.0001/max * i) ,std::sin(0.0001/max * i) );
            }
            ComputeMVBB::Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(7,t);
        }


        {
            // generate points
            Vector2List v;
            v.push_back(Vector2(1,0));

            ComputeMVBB::Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(8,t);
        }

        {
            // generate points
            Vector2List v;

            ComputeMVBB::Matrix2Dyn t(2,v.size());
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
            ComputeMVBB::Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(10,t);
        }


        {
            // generate points
            ComputeMVBB::Matrix2Dyn t(2,10000000);
            t.setRandom();
            minRectTest(11,t);
        }


        {
            // generate points
            auto v = getPointsFromFile2D("./PointsSimulation2DRectFail.txt");
            ComputeMVBB::Matrix2Dyn t(2,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            minRectTest(12,t);
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
    void diameterTest(unsigned int N, const TMatrix & v, bool dump = true, unsigned int optLoops = 10, PREC epsilon = 0.001) {
        using namespace PointFunctions;
        using namespace TestFunctions;

        if(dump) {
            dumpPointsMatrix("./DiameterTest" + std::to_string(N) +".txt",v);
        }

        std::cout << "\n\nStart approximateMVBBDiam Test "+ std::to_string(N) +"" << std::endl;
        START_TIMER(start)
        auto oobb = ComputeMVBB::approximateMVBBDiam(v,epsilon,optLoops);
        STOP_TIMER_SEC(count, start)
        std::cout << "Timings: " << count << " sec for " <<v.cols() << " points" << std::endl;
        std::cout << "End approximateMVBBDiam Test "+ std::to_string(N) << std::endl;

        oobb.expand(1e-10);
        if(!checkPointsInOOBB(v,oobb)){
            std::cout << "WARNING: Not all points in OOBB.expand(1e-10)" << std::endl;
        }else{
            std::cout << "All points in OOBB!" << std::endl;
        }

        std::cout << "Start Sampling Test "+ std::to_string(N) +"" << std::endl;
        Matrix3Dyn sampled;
        START_TIMER(start2)
        ComputeMVBB::samplePointsGrid(sampled,v,400,oobb);
        STOP_TIMER_SEC(count2, start2)
        std::cout << "Timings: " << count2 << " sec for " <<sampled.cols() << " points" << std::endl;
        std::cout << "End Sampling Test "+ std::to_string(N) << std::endl;

        dumpOOBB("./DiameterTest"+ std::to_string(N) +"Out.txt", oobb);

        dumpPointsMatrix("./DiameterTest"+ std::to_string(N) +"Out2.txt",sampled);

    }


    void test() {
        using namespace PointFunctions;
        using namespace TestFunctions;
        {
            // generate points
            ComputeMVBB::Matrix3Dyn t(3,500);
            t.setRandom();
            diameterTest(1,t);
        }

        {
            // generate points
            ComputeMVBB::Matrix3Dyn t(3,10000);
            t.setRandom();
            diameterTest(2,t);
        }

        {
            // generate points
            ComputeMVBB::Matrix3Dyn t(3,140000000);
            t.setRandom();
            diameterTest(3,t,false,0,0.01);
        }

        {
            // generate points
            auto v = getPointsFromFile3D("./PointsSimulation.txt");

            ComputeMVBB::Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            diameterTest(3,t);
        }

        {
            // generate points
            auto v = getPointsFromFile3D("./PointsSimulationFailMVBB.txt");
            ComputeMVBB::Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            diameterTest(4,t,true,10);
        }

        {
            // generate points
            auto v = getPointsFromFile3D("./Lucy.txt");

            ComputeMVBB::Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            diameterTest(5,t,false,3,100);
        }

         {
            // generate points
            auto v = getPointsFromFile3D("./Bunny.txt");

            ComputeMVBB::Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            diameterTest(6,t,false,10,1);
        }
    }
};


void diameterTest() {
    srand(time(NULL));
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
        }

        std::cout << "\n\nStart MVBBTest Test "+ std::to_string(N) +"" << std::endl;
        START_TIMER(start)
        auto oobb = ComputeMVBB::approximateMVBB(v,eps,nPoints,gridSize,mvbbDiamOptLoops,gridSearchOptLoops);
        STOP_TIMER_SEC(count, start)
        std::cout << "Timings: " << count << " sec for " <<v.cols() << " points" << std::endl;
        std::cout << "End MVBBTest Test "+ std::to_string(N) + "Out.txt" << std::endl;


        dumpOOBB("./MVBBTest"+ std::to_string(N) +"Out.txt", oobb);

    }

    void test() {
        using namespace PointFunctions;
        using namespace TestFunctions;
        {
            // generate points
            auto v = generatePoints3D(100);

            ComputeMVBB::Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            mvbbTest(1,t);
        }

        {
            // generate points

            auto v = generatePoints3D(10000);

            ComputeMVBB::Matrix3Dyn t(3,v.size());
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

//        {
              // generate points
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

            ComputeMVBB::Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }
            applyRandomRotTrans(t);
            mvbbTest(3,t,true,0.1);
        }

        {
            // generate points
            auto v = getPointsFromFile3D("./Lucy.txt");
            ComputeMVBB::Matrix3Dyn t(3,v.size());
            for(unsigned int i = 0; i<v.size(); ++i) {
                t.col(i) = v[i];
            }

            applyRandomRotTrans(t);
            mvbbTest(4,t,false,100,400,5,0,5);
        }

    }
};


void mvbbTest() {
    srand(0);
    MVBBTests t;
    t.test();
}

#endif
