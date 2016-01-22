// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================
#ifndef TestFunctions_hpp
#define TestFunctions_hpp

#include <stdlib.h>
#include <fstream>
#include <functional>

#include <gtest/gtest.h>

#include "ApproxMVBB/Config/Config.hpp"
#include "ApproxMVBB/Common/AssertionDebug.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE

#include "ApproxMVBB/PointFunctions.hpp"
#include "ApproxMVBB/OOBB.hpp"


namespace ApproxMVBB{

namespace TestFunctions{

    ApproxMVBB_DEFINE_MATRIX_TYPES
    ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES

    static const int randomSeed = 314159;

    // TODO not std lib conform!
    std::size_t setRandomSeedStd(std::string name);

    template<typename A, typename B>
    ::testing::AssertionResult assertNearArrays(  const char * a_expr,
                                                  const char * b_expr,
                                                  const A & a,
                                                  const B & b,
                                                  PREC absError = 1e-6)
    {
        using ScalarA = typename A::Scalar;
        using ScalarB = typename B::Scalar;

        ApproxMVBB_STATIC_ASSERTM( (std::is_same<ScalarA,ScalarB>::value) , "Precision not the same!")

        if(a.size() != b.size() ){
            return ::testing::AssertionFailure() << "A and B not the same size";
        }

        const ScalarA * pA = a.data();
        const ScalarA * pAend = a.data() + a.size();
        const ScalarB * pB = b.data();
        for(;pA != pAend; ++pA, ++pB){
            if( std::abs(*pA - *pB) >= absError){
                return ::testing::AssertionFailure() << "A and B not the same size: " << *pA << " != " << *pB << " absTol: " << absError;
            }
        }
        return ::testing::AssertionSuccess();
    }


    template<typename Derived>
    void dumpPointsMatrix(std::string filePath, const MatrixBase<Derived> & v) {

        std::ofstream l;
        l.open(filePath.c_str());
        if(!l.good()){
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
        }

        if(v.cols() != 0){
            unsigned int i=0;
            for(; i<v.cols()-1; i++) {
                l << v.col(i).transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
            }
            l << v.col(i).transpose().format(MyMatrixIOFormat::SpaceSep);
        }

        l.close();
    }


    int isBigEndian(void);

    template <typename T>
    T swapEndian(T u)
    {
        ApproxMVBB_STATIC_ASSERTM(sizeof(char) == 1, "char != 8 bit");

        union
        {
            T u;
            unsigned char u8[sizeof(T)];
        } source, dest;

        source.u = u;

        for (size_t k = 0; k < sizeof(T); k++)
            dest.u8[k] = source.u8[sizeof(T) - k - 1];

        return dest.u;
    }

    template<class Matrix>
    void dumpPointsMatrixBinary(std::string filename, const Matrix& matrix){
        std::ofstream out(filename,std::ios::out | std::ios::binary | std::ios::trunc);
        typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
        ApproxMVBB_STATIC_ASSERT( sizeof(typename Matrix::Index) == 8 )
        char bigEndian = isBigEndian();

        out.write((char*) (&bigEndian), sizeof(char));
        out.write((char*) (&rows), sizeof(typename Matrix::Index));
        out.write((char*) (&cols), sizeof(typename Matrix::Index));

        typename Matrix::Index bytes = sizeof(typename Matrix::Scalar);
        out.write((char*) (&bytes), sizeof(typename Matrix::Index ));

        out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
        out.close();
    }
    template<class Matrix>
    void readPointsMatrixBinary(std::string filename, Matrix& matrix, bool withHeader=true){
        std::ifstream in(filename,std::ios::in | std::ios::binary);
        typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
        ApproxMVBB_STATIC_ASSERT( sizeof(typename Matrix::Index) == 8 )
        char bigEndian = 0; // assume all input files with no headers are little endian!

        if(withHeader){
            in.read((char*) (&bigEndian), sizeof(char));
            in.read((char*) (&rows),sizeof(typename Matrix::Index));
            in.read((char*) (&cols),sizeof(typename Matrix::Index));
            typename Matrix::Index bytes;
            in.read((char*) (&bytes), sizeof(typename Matrix::Index ));
            if(bytes != sizeof(typename Matrix::Scalar)){
                ApproxMVBB_ERRORMSG("read binary with wrong data type: " << filename << " Scalar bytes: " << bytes);
            }

            // swap endianness if file has other type
            if(isBigEndian() != bigEndian ){
                rows = swapEndian(rows);
                cols = swapEndian(cols);
                bytes = swapEndian(bytes);
            }
            matrix.resize(rows, cols);
        }
        in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );

        // swap endianness of whole matrix if file has other type
        if(isBigEndian() != bigEndian ){
            auto f = [](const typename Matrix::Scalar & v){return swapEndian(v);};
            matrix = matrix.unaryExpr( f );
        }
        in.close();
    }

    template<typename Container>
    void dumpPoints(std::string filePath, Container & c) {

        std::ofstream l;
        l.open(filePath.c_str());
        if(!l.good()){
            ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
        }
        if(c.size()!=0){
            auto e = c.begin() + (c.size() - 1);
            auto it = c.begin();
            for(; it != e; ++it) {
                l << it->transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
            }
            l << (it)->transpose().format(MyMatrixIOFormat::SpaceSep);
        }
        l.close();
    }

    void readOOBB(std::string filePath, Vector3 & minP, Vector3 & maxP, Matrix33 & R_KI,  Vector3List & pointList);
    void readOOBBAndCheck( OOBB & validOOBB, std::string filePath);
    void dumpOOBB(std::string filePath, const OOBB & oobb);

    Vector3List getPointsFromFile3D(std::string filePath);

    Vector2List getPointsFromFile2D(std::string filePath);

//    Vector2List generatePoints2D(unsigned int n=4);
//
//    Vector3List generatePoints3D(unsigned int n=4);

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

}
}

#endif
