// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Common_MyMatrixTypeDefs_hpp
#define ApproxMVBB_Common_MyMatrixTypeDefs_hpp

#include "ApproxMVBB/Common/Platform.hpp"

//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>

namespace ApproxMVBB{

// ================================================================================================
/** @brief This
*	These are some small matrix definitions.
*/

namespace MyMatrix {

    template<typename Scalar>
    using  Matrix44 =  Eigen::Matrix<Scalar, 4, 4>;
    template<typename Scalar>
    using  Matrix43 =  Eigen::Matrix<Scalar, 4, 3>;
    template<typename Scalar>
    using  Matrix34 =  Eigen::Matrix<Scalar, 3, 4>;
    template<typename Scalar>
    using  Matrix33 =  Eigen::Matrix<Scalar, 3, 3>;
    template<typename Scalar>
    using  Matrix32 =  Eigen::Matrix<Scalar, 3, 2>;
    template<typename Scalar>
    using  Matrix23 =  Eigen::Matrix<Scalar, 2, 3>;
    template<typename Scalar>
    using  Matrix22 =  Eigen::Matrix<Scalar, 2, 2>;
    template<typename Scalar>
    using  Vector3 =  Eigen::Matrix<Scalar, 3, 1>;
    template<typename Scalar>
    using  Vector2 =  Eigen::Matrix<Scalar, 2, 1>;

    template<typename Scalar>
    using  Quaternion =  Eigen::Quaternion<Scalar>;
    template<typename Scalar>
    using  AngleAxis =  Eigen::AngleAxis<Scalar>;

    template<typename Scalar>
    using  Vector4 =  Eigen::Matrix<Scalar, 4, 1>;
    template<typename Scalar>
    using  Vector6 =  Eigen::Matrix<Scalar, 6, 1>;
    template<typename Scalar>
    using  VectorDyn =  Eigen::Matrix<Scalar, Eigen::Dynamic , 1 >;


    template<typename Scalar>
    using  MatrixDynDyn =  Eigen::Matrix<Scalar, Eigen::Dynamic , Eigen::Dynamic >;
    template<typename Scalar>
    using  MatrixDiagDyn =  Eigen::DiagonalMatrix<Scalar, Eigen::Dynamic >;
    template<typename Scalar>
    using  MatrixDynDynRow =  Eigen::Matrix<Scalar, Eigen::Dynamic , Eigen::Dynamic, Eigen::RowMajor>;

    template<typename Scalar,  int M>
    using  MatrixStatDyn =  Eigen::Matrix<Scalar, M, Eigen::Dynamic >;
    template<typename Scalar, int N>
    using  MatrixDynStat =  Eigen::Matrix<Scalar, Eigen::Dynamic, N >;
    template<typename Scalar, int M,  int N>
    using  MatrixStatStat =  Eigen::Matrix<Scalar, M, N >;
    template<typename Scalar, int M>
    using  VectorStat =  Eigen::Matrix<Scalar, M, 1 >;


    template<typename Scalar>
    using  AffineTrafo =  Eigen::Transform<Scalar,3,Eigen::TransformTraits::Affine>;
    template<typename Scalar>
    using  AffineTrafo2d =  Eigen::Transform<Scalar,2,Eigen::TransformTraits::Affine>;

    template<typename Scalar, int M>
    using  ArrayStatDyn =  Eigen::Array<Scalar, M, Eigen::Dynamic >;
    template<typename Scalar, int N>
    using  ArrayDynStat =  Eigen::Array<Scalar, Eigen::Dynamic, N >;
    template<typename Scalar, int M,  int N>
    using  ArrayStatStat =  Eigen::Array<Scalar, M, N >;
    template<typename Scalar, int M>
    using  ArrayStat =  Eigen::Array<Scalar, M,1>;

    template<typename Scalar>
    using  Array3 =  Eigen::Array<Scalar, 3, 1>;
    template<typename Scalar>
    using  Array2 =  Eigen::Array<Scalar, 2, 1>;

};

namespace MyMatrix{

    template<typename Derived> using MatrixBase = Eigen::MatrixBase<Derived>;
    template<typename Derived> using ArrayBase = Eigen::ArrayBase<Derived>;

    template<typename Derived>                 using VectorBDyn     = Eigen::VectorBlock<Derived,Eigen::Dynamic>;
    template<typename Derived, unsigned int M> using VectorBStat    = Eigen::VectorBlock<Derived,M>;

    template<typename Derived>                 using MatrixBDynDyn  = Eigen::Block<Derived>;
    template<typename Derived, unsigned int M> using MatrixBStatDyn = Eigen::Block<Derived,M, Eigen::Dynamic>;
    template<typename Derived, unsigned int N> using MatrixBDynStat = Eigen::Block<Derived,Eigen::Dynamic,N>;

    template<typename EigenType> using MatrixRef = Eigen::Ref<EigenType>;

    template<typename EigenType> using MatrixMap = Eigen::Map<EigenType>;

};


struct APPROXMVBB_EXPORT  MyMatrixIOFormat {
     static const  Eigen::IOFormat Matlab;
     static const  Eigen::IOFormat CommaSep;
     static const  Eigen::IOFormat SpaceSep;
};

}


#define DEFINE_MATRIX_SPECIALTYPES \
   template<typename Derived> using MatrixBase = typename MyMatrix::MatrixBase<Derived>; \
   template<typename Derived> using ArrayBase  = typename MyMatrix::ArrayBase<Derived>; \
   \
   template<typename Derived> using VectorBDyn = typename MyMatrix::VectorBDyn<Derived>; \
   template<typename Derived,unsigned int M> using VectorBStat = typename MyMatrix::VectorBStat<Derived,M>; \
   \
   template<typename Derived> using MatrixBDynDyn = typename MyMatrix::MatrixBDynDyn<Derived>; \
   template<typename Derived, unsigned int N> using MatrixBDynStat = typename MyMatrix::MatrixBDynStat<Derived,N>; \
   template<typename Derived, unsigned int M> using MatrixBStatDyn = typename MyMatrix::MatrixBStatDyn<Derived,M>; \
    \
   template<typename EigenType> using MatrixRef = typename MyMatrix::MatrixRef< EigenType >; \
   template<typename EigenType> using MatrixMap = typename MyMatrix::MatrixMap< EigenType >; \

/**
* @brief This macro is used to typedef all custom matrix types which have nothing to do with the system.
*/
#define ApproxMVBB_DEFINE_MATRIX_TYPES_OF( _PREC_ ) \
    using Matrix44 = typename MyMatrix::Matrix44< _PREC_ >; \
    using Matrix33 = typename MyMatrix::Matrix33< _PREC_ >; \
    using Matrix22 = typename MyMatrix::Matrix22< _PREC_ >; \
    using Matrix32 = typename MyMatrix::Matrix32< _PREC_ >; \
    using Matrix23 = typename MyMatrix::Matrix23< _PREC_ >; \
    using Matrix43 = typename MyMatrix::Matrix43< _PREC_ >; \
    using Matrix34 = typename MyMatrix::Matrix34< _PREC_ >; \
    using Vector3 = typename MyMatrix::Vector3< _PREC_ >; \
    using Vector2 = typename MyMatrix::Vector2< _PREC_ >; \
    using Vector4 = typename MyMatrix::Vector4< _PREC_ >; \
    using Vector6 = typename MyMatrix::Vector6< _PREC_ >; \
    using Quaternion = typename MyMatrix::Quaternion< _PREC_ >; \
    using AngleAxis = typename MyMatrix::AngleAxis< _PREC_ >; \
    using VectorDyn = typename MyMatrix::VectorDyn< _PREC_ >; \
    using MatrixDynDyn = typename MyMatrix::MatrixDynDyn< _PREC_ >; \
    using MatrixDiagDyn = typename MyMatrix::MatrixDiagDyn< _PREC_ >; \
    using MatrixDynDynRow = typename MyMatrix::MatrixDynDynRow< _PREC_ >; \
    \
    template<unsigned int M> using MatrixStatDyn = typename MyMatrix::MatrixStatDyn< _PREC_, M>; \
    template<unsigned int N> using MatrixDynStat = typename MyMatrix::MatrixDynStat< _PREC_, N>; \
    template<unsigned int M,unsigned int N> using MatrixStatStat = typename MyMatrix::MatrixStatStat< _PREC_, M,N>; \
    template<unsigned int M> using VectorStat = typename MyMatrix::VectorStat< _PREC_, M>; \
    \
    using AffineTrafo = typename MyMatrix::AffineTrafo< _PREC_ >; \
    using AffineTrafo2d = typename MyMatrix::AffineTrafo2d< _PREC_ >; \
    \
    template<unsigned int M> using ArrayStatDyn = typename MyMatrix::ArrayStatDyn< _PREC_, M>; \
    template<unsigned int N> using ArrayDynStat = typename MyMatrix::ArrayDynStat< _PREC_, N>; \
    template<unsigned int M,unsigned int N> using ArrayStatStat = typename MyMatrix::ArrayStatStat< _PREC_, M,N>; \
    template<unsigned int M> using ArrayStat = typename MyMatrix::ArrayStat< _PREC_, M>; \
    using Array3 = typename MyMatrix::Array3< _PREC_ >; \
    using Array2 = typename MyMatrix::Array2< _PREC_ >; \
    \
    DEFINE_MATRIX_SPECIALTYPES

#endif

