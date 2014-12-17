// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================
#ifndef TestFunctions_hpp
#define TestFunctions_hpp

#include <fstream>


#include "ApproxMVBB/Config/Config.hpp"
#include "ApproxMVBB/Common/AssertionDebug.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE

#include "ApproxMVBB/PointFunctions.hpp"
#include "ApproxMVBB/OOBB.hpp"


namespace ApproxMVBB{

namespace TestFunctions{

    ApproxMVBB_DEFINE_MATRIX_TYPES
    DEFINE_POINTS_CONFIG_TYPES


    template<typename Derived>
    void dumpPointsMatrix(std::string filePath, const MatrixBase<Derived> & v) {

        std::ofstream l;
        l.open(filePath.c_str());
        if(!l.good()){
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
        }

        for(unsigned int i=0; i<v.cols(); i++) {
            l << v.col(i).transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        }
    }

    template<int M>
    void dumpPointsMatrixBinary(std::string filePath, const MatrixStatDyn<M> & v) {

        std::ofstream l;
        l.open(filePath.c_str(),std::ios::binary);
        if(!l.good()){
            ApproxMVBB_ERRORMSG("Could not open binary file: " << filePath << std::endl)
        }

        l.write(reinterpret_cast<const char*>(v.data()), v.size()*sizeof(PREC));
    }

    template<typename Container>
    void dumpPoints(std::string filePath, Container & c) {

        std::ofstream l;
        l.open(filePath.c_str());
        if(!l.good()){
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
        }

        for(auto & v: c) {
            l << v.transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        }
    }

    void dumpOOBB(std::string filePath, OOBB & oobb);

    template<int M>
    void getPointsFromFileBinary(std::string filePath,  MatrixStatDyn<M> & v) {
        std::ifstream l;
        l.open(filePath.c_str(),std::ios::binary);
        if(!l.good()){
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
        }

       l.read(reinterpret_cast<char*>(v.data()), v.size()*sizeof(PREC));

    }

    PointFunctions::Vector3List getPointsFromFile3D(std::string filePath);

    Vector2List getPointsFromFile2D(std::string filePath);

    Vector2List generatePoints2D(unsigned int n=4);

    Vector3List generatePoints3D(unsigned int n=4);

    template<typename Derived, typename IndexSet>
    Derived filterPoints(MatrixBase<Derived> & v, IndexSet & s){

        Derived ret;
        ret.resize(v.rows(),s.size());

        auto size = v.cols();
        decltype(size) k = 0;
        for(auto i : s){
            if(i < size && i >=0){
                ret.col(k++) = v.col(i);
            }
        };
        return ret;
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

};

#endif
