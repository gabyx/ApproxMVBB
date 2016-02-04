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

    std::size_t hashString(std::string s){
        std::size_t h = 3141459;
        for(char & c : s )
        {
            h = h * 101  +  c;
        }
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

    OOBB readOOBBAndCheck( OOBB & oobb, std::string filePath){
            Vector3List points(8);
            Vector3 minP;
            Vector3 maxP;
            Matrix33 R_KI;
            readOOBB(filePath, minP, maxP, R_KI, points);

            Vector3List pointsV = oobb.getCornerPoints();


            Matrix3Dyn  pV(3,pointsV.size());
            Matrix3Dyn  p(3,points.size());

            for(unsigned int i=0;i<pointsV.size();++i){
               pV.col(i)= pointsV[i];
            }
            for(unsigned int i=0;i<points.size();++i){
               p.col(i) = points[i];
            }

            std::cout << "Compare Corners: valid:" << std::endl;
            std::cout << p << std::endl;
            std::cout << "and:" << std::endl;
            std::cout << pV << std::endl;

            EXPECT_TRUE( assertNearArrayColsRows<true>(p, pV) ) << "Not all corner points of OOBB equal to validation OOBB in file: " << filePath;

            std::cout << filePath << std::endl
            <<"valid minP: " << std::endl << minP.transpose() << " and " << std::endl << oobb.m_minPoint.transpose() << std::endl
            <<"valid maxP: " << std::endl << maxP.transpose() << " and " << std::endl  << oobb.m_maxPoint.transpose() << std::endl
            <<"valid R_IK: " << std::endl << R_KI  << " and " << std::endl << oobb.m_q_KI.matrix() << std::endl;

            // return valid oobb
            return {minP,maxP,R_KI};
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
