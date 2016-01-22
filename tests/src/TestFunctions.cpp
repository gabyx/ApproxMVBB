// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include "TestFunctions.hpp"

#include <random>

namespace ApproxMVBB{
namespace TestFunctions{

//    static const int randomSeed = 314159;

    std::size_t setRandomSeedStd(std::string name){
        auto h = std::hash<std::string>{}(name);
        srand48(h);
        srand(h);
        return h;
    }

    int isBigEndian(void)
    {
        union {
            uint32_t i;
            char c[4];
        } bint = {0x01020304};

        return bint.c[0] == 1;
    }


    void dumpOOBB(std::string filePath, const OOBB & oobb) {

        std::ofstream l;
        l << std::setprecision(12);
        l.open(filePath.c_str());
        if(!l.good()){
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
        }

        l << oobb.m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        l << oobb.m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        l << oobb.m_q_KI.matrix().format(MyMatrixIOFormat::SpaceSep);

        // Export all corner points in I system
        auto points = oobb.getCornerPoints();
        for(auto & p : points){
            l << std::endl << p.transpose().format(MyMatrixIOFormat::SpaceSep);
        }
        l.close();
    }

    void readOOBB(std::string filePath, Vector3 & minP, Vector3 & maxP, Matrix33 & R_KI, Vector3List & pointList) {
          auto v = getPointsFromFile3D(filePath);
          if(v.size()!=5 + 8){
              ApproxMVBB_ERRORMSG("Wrong number of points in OOBB file: " << filePath << " points: " << v.size())
          }
          minP = v[0];
          maxP = v[1];
          R_KI.row(0) = v[2];
          R_KI.row(1) = v[3];
          R_KI.row(2) = v[4];

          pointList.resize(8);
          for(unsigned int i =0; i< 8 ; ++i){
            pointList[i] = v[5+i];
          }

    }

    void readOOBBAndCheck( OOBB & validOOBB, std::string filePath){
            Vector3List points(8);
            Vector3 minP;
            Vector3 maxP;
            Matrix33 R_KI;
            readOOBB(filePath, minP, maxP, R_KI, points);

            Vector3List pointsValid = validOOBB.getCornerPoints();

            // Check all 8 points
            bool indexMatched[8] = {false};
            unsigned int nMatched = 0;
            unsigned int i = 0;
            unsigned int pointIdx = 0;
            while( pointIdx < 8){
                if( i < 8){
                    if( !indexMatched[i] ){
                        // check points[pointIdx] against i-th valid one
                        if ( assertNearArrays("points[pointIdx]", "pointsValid[i]", points[pointIdx],pointsValid[i]) ){
                            indexMatched[i] = true; ++nMatched;
                        }
                    }else{
                        ++i;
                        continue;
                    }
                }

                // all indices i checked go to next point, reset check idx
                i = 0;
                ++pointIdx;
            }

            ASSERT_EQ(nMatched, 8) << "Not all points of OOBB equal to validation OOBB in file: "
            << filePath << std::endl
            <<"valid minP: " << std::endl << minP.transpose() << " and " << std::endl << validOOBB.m_minPoint.transpose() << std::endl
            <<"valid maxP: " << std::endl << maxP.transpose() << " and " << std::endl  << validOOBB.m_maxPoint.transpose() << std::endl
            <<"valid R_IK: " << std::endl << R_KI  << " and " << std::endl << validOOBB.m_q_KI.matrix() << std::endl;
    }


    PointFunctions::Vector3List getPointsFromFile3D(std::string filePath) {

        std::ifstream file;            //creates stream myFile
        file.open(filePath.c_str());  //opens .txt file

        if (!file.is_open()) { // check file is open, quit if not
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath)
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
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath)
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

//     Vector2List generatePoints2D(unsigned int n) {
//        Vector2List v;
//        for(unsigned int i=0; i<n; i++) {
//            v.push_back( Vector2::Random() );
//        }
//        return v;
//    }
//
//    Vector3List generatePoints3D(unsigned int n) {
//        Vector3List v;
//        for(unsigned int i=0; i<n; i++) {
//            v.push_back( Vector3::Random() );
//        }
//        return v;
//    }
}
}
