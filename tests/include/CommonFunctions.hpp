// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================
#ifndef FileFunctions_hpp
#define FileFunctions_hpp

#include <fstream>
#include "ApproxMVBB/Common/AssertionDebug.hpp"
#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE

#include "ApproxMVBB/OOBB.hpp"
#include "ApproxMVBB/PointFunctions.hpp"

namespace ApproxMVBB
{
namespace TestFunctions
{
ApproxMVBB_DEFINE_MATRIX_TYPES;
ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES;

// TODO not std lib conform!
std::size_t hashString(std::string name);

template <typename T = PREC>
std::string getPrecAbrev();

std::string getFileInPath(std::string name);
std::string getFileInAddPath(std::string name);
std::string getPointsDumpPath(std::string name, std::string suffix = ".bin");
std::string getFileOutPath(std::string name, std::string suffix = ".bin");
std::string getFileValidationPath(std::string name, std::string suffix = ".bin");

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

    for (size_t k  = 0; k < sizeof(T); k++)
        dest.u8[k] = source.u8[sizeof(T) - k - 1];

    return dest.u;
}

void readOOBB(std::string filePath, Vector3& minP, Vector3& maxP, Matrix33& R_KI, Vector3List& pointList);
OOBB readOOBBAndCheck(OOBB& oobb, std::string filePath);
void dumpOOBB(std::string filePath, const OOBB& oobb);

Vector3List getPointsFromFile3D(std::string filePath);
Vector2List getPointsFromFile2D(std::string filePath);

template <typename Container>
void dumpPoints(std::string filePath, Container& c)
{
    std::ofstream l;
    l.open(filePath.c_str());
    if (!l.good())
    {
        ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
    }
    if (c.size() != 0)
    {
        auto e  = c.begin() + (c.size() - 1);
        auto it = c.begin();
        for (; it != e; ++it)
        {
            l << it->transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        }
        l << (it)->transpose().format(MyMatrixIOFormat::SpaceSep);
    }
    l.close();
}
template <typename Derived>
void dumpPointsMatrix(std::string filePath, const MatrixBase<Derived>& v)
{
    std::ofstream l;
    l.open(filePath.c_str());
    if (!l.good())
    {
        ApproxMVBB_ERRORMSG("Could not open file: " << filePath << std::endl)
    }

    if (v.cols() != 0)
    {
        unsigned int i = 0;
        for (; i < v.cols() - 1; i++)
        {
            l << v.col(i).transpose().format(MyMatrixIOFormat::SpaceSep) << std::endl;
        }
        l << v.col(i).transpose().format(MyMatrixIOFormat::SpaceSep);
    }

    l.close();
}

template <class Matrix>
void dumpPointsMatrixBinary(std::string filename, const Matrix& matrix)
{
    std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
    typename Matrix::Index rows                                                  = matrix.rows();
    typename Matrix::Index cols                                                  = matrix.cols();
    ApproxMVBB_STATIC_ASSERT(sizeof(typename Matrix::Index) == 8) bool bigEndian = isBigEndian();

    out.write((char*)(&bigEndian), sizeof(bool));
    out.write((char*)(&rows), sizeof(typename Matrix::Index));
    out.write((char*)(&cols), sizeof(typename Matrix::Index));

    typename Matrix::Index bytes = sizeof(typename Matrix::Scalar);
    out.write((char*)(&bytes), sizeof(typename Matrix::Index));

    out.write((char*)matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));
    out.close();
}
template <class Matrix>
void readPointsMatrixBinary(std::string filename, Matrix& matrix, bool withHeader = true)
{
    std::ifstream in(filename, std::ios::in | std::ios::binary);

    if (!in.is_open())
    {
        ApproxMVBB_ERRORMSG("cannot open file: " << filename);
    }

    typename Matrix::Index rows = matrix.rows();
    typename Matrix::Index cols = matrix.cols();
    ApproxMVBB_STATIC_ASSERT(sizeof(typename Matrix::Index) == 8) bool bigEndian =
        false;  // assume all input files with no headers are little endian!

    if (withHeader)
    {
        in.read((char*)(&bigEndian), sizeof(bool));
        in.read((char*)(&rows), sizeof(typename Matrix::Index));
        in.read((char*)(&cols), sizeof(typename Matrix::Index));
        typename Matrix::Index bytes;
        in.read((char*)(&bytes), sizeof(typename Matrix::Index));

        // swap endianness if file has other type
        if (isBigEndian() != bigEndian)
        {
            rows  = swapEndian(rows);
            cols  = swapEndian(cols);
            bytes = swapEndian(bytes);
        }
        if (bytes != sizeof(typename Matrix::Scalar))
        {
            ApproxMVBB_ERRORMSG(
                "read binary with wrong data type: " << filename << "bigEndian: " << bigEndian << ", rows: " << rows
                                                     << ", cols: "
                                                     << cols
                                                     << ", scalar bytes: "
                                                     << bytes);
        }

        matrix.resize(rows, cols);
    }
    in.read((char*)matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));

    // swap endianness of whole matrix if file has other type
    if (isBigEndian() != bigEndian)
    {
        auto f = [](const typename Matrix::Scalar& v) { return swapEndian(v); };
        matrix = matrix.unaryExpr(f);
    }
    in.close();
}
}
}

#endif
