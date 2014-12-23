// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include "TestFunctions.hpp"


namespace ApproxMVBB{
namespace TestFunctions{

    void dumpOOBB(std::string filePath, OOBB & oobb) {

        std::ofstream l;
        l.open(filePath.c_str());
        if(!l.good()){
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
        }

        l << oobb.m_minPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        l << oobb.m_maxPoint.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        l << oobb.m_q_KI.matrix().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        l.close();
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

     Vector2List generatePoints2D(unsigned int n) {
        Vector2List v;
        for(unsigned int i=0; i<n; i++) {
            v.push_back( Vector2::Random() );
        }
        return v;
    }

    Vector3List generatePoints3D(unsigned int n) {
        Vector3List v;
        for(unsigned int i=0; i<n; i++) {
            v.push_back( Vector3::Random() );
        }
        return v;
    }
};
};
