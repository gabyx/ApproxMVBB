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

#include <vector>
#include <map>
//#include <Eigen/StdVector>


#include <Eigen/Dense>

namespace ApproxMVBB{

// ================================================================================================
/** @brief This
*	These are some small matrix definitions.
*/
template<typename TPREC>
struct MyMatrix {
    using PREC = TPREC;
    //Static assigned Matrices
    using Matrix44 = Eigen::Matrix<PREC, 4, 4>;
    using Matrix33 = Eigen::Matrix<PREC, 3, 3>;
    using Matrix22 = Eigen::Matrix<PREC, 2, 2>;
    using Vector3 = Eigen::Matrix<PREC, 3, 1>;
    using Vector2 = Eigen::Matrix<PREC, 2, 1>;

    using Quaternion = Eigen::Quaternion<PREC>;
    using AngleAxis = Eigen::AngleAxis<PREC>;

    using VectorDyn = Eigen::Matrix<PREC, Eigen::Dynamic , 1 >                   ;



    using MatrixDynDyn = Eigen::Matrix<PREC, Eigen::Dynamic , Eigen::Dynamic >      ;
    using MatrixDiagDyn = Eigen::DiagonalMatrix<PREC, Eigen::Dynamic >               ;
    using MatrixDynDynRow = Eigen::Matrix<PREC, Eigen::Dynamic , Eigen::Dynamic, Eigen::RowMajor>;

    template<int M>
    using MatrixStatDyn = Eigen::Matrix<PREC, M, Eigen::Dynamic >;
    template<int N>
    using MatrixDynStat = Eigen::Matrix<PREC, Eigen::Dynamic, N >;

    template<unsigned int M, unsigned int N>
    using MatrixStatStat = Eigen::Matrix<PREC, M, N >;

    template<unsigned int M>
    using VectorStat = Eigen::Matrix<PREC, M, 1 >;

    template<typename Derived> using MatrixBase = Eigen::MatrixBase<Derived>;
    template<typename Derived> using MatrixDenseBase = Eigen::DenseBase<Derived>;

    template<typename EigenType> using MatrixRef = Eigen::Ref<EigenType>;
    template<typename EigenType> using MatrixMap = Eigen::Map<EigenType>;


    // Sepcial STL map where the type is 16byte aligned
    template<typename Key, typename Type, typename Comp = std::less<Key> >
    using StdMapAligned = std::map<Key, Type, Comp, Eigen::aligned_allocator<std::pair<const Key, Type> > >;

     // Special STL vectors where the type is 16byte aligned
    template<typename Type >
    using StdVecAligned = std::vector<Type, Eigen::aligned_allocator<Type> >;

    // Special Array types;
    template<typename Derived> using ArrayBase  = Eigen::ArrayBase<Derived>;
    template<unsigned int M>
    using ArrayStatDyn = Eigen::Array<PREC, M, Eigen::Dynamic >;
    template<unsigned int N>
    using ArrayDynStat = Eigen::Array<PREC, Eigen::Dynamic, N >;
    template<unsigned int M, unsigned int N>
    using ArrayStatStat = Eigen::Array<PREC, M, N >;
    template<unsigned int M>
    using ArrayStat = Eigen::Array<PREC, M,1>;

    using Array3 = Eigen::Array<PREC, 3, 1>;
    using Array2 = Eigen::Array<PREC, 2, 1>;

    using AffineTrafo = Eigen::Transform<PREC,3,Eigen::TransformTraits::Affine>;
    using AffineTrafo2d = Eigen::Transform<PREC,2,Eigen::TransformTraits::Affine>;
};


struct APPROXMVBB_EXPORT  MyMatrixIOFormat {
     static const  Eigen::IOFormat Matlab;
     static const  Eigen::IOFormat CommaSep;
     static const  Eigen::IOFormat SpaceSep;
};

};


/**
* @brief This macro is used to typedef all custom matrix types which have nothing to do with the system.
*/
#define ApproxMVBB_DEFINE_MATRIX_TYPES_OF( _PREC_ ) \
   using Matrix44 = typename ApproxMVBB::MyMatrix< _PREC_ >::Matrix44; \
   using Matrix33 = typename ApproxMVBB::MyMatrix< _PREC_ >::Matrix33; \
   using Matrix22 = typename ApproxMVBB::MyMatrix< _PREC_ >::Matrix22; \
   using Vector3 = typename ApproxMVBB::MyMatrix< _PREC_ >::Vector3;   \
   using Vector2 = typename ApproxMVBB::MyMatrix< _PREC_ >::Vector2;   \
   using Quaternion = typename ApproxMVBB::MyMatrix< _PREC_ >::Quaternion; \
   using AngleAxis = typename ApproxMVBB::MyMatrix< _PREC_ >::AngleAxis; \
   using VectorDyn = typename ApproxMVBB::MyMatrix< _PREC_ >::VectorDyn; \
   using MatrixDynDyn = typename ApproxMVBB::MyMatrix< _PREC_ >::MatrixDynDyn; \
   using MatrixDiagDyn = typename ApproxMVBB::MyMatrix< _PREC_ >::MatrixDiagDyn; \
   using MatrixDynDynRow = typename ApproxMVBB::MyMatrix< _PREC_ >::MatrixDynDynRow; \
   \
   template<int M> using MatrixStatDyn = typename ApproxMVBB::MyMatrix< _PREC_ >::template MatrixStatDyn<M>; \
   template<int N> using MatrixDynStat = typename ApproxMVBB::MyMatrix< _PREC_ >::template MatrixDynStat<N>; \
   template<unsigned int M,unsigned int N> using MatrixStatStat = typename ApproxMVBB::MyMatrix< _PREC_ >::template MatrixStatStat<M,N>; \
   template<unsigned int M> using VectorStat = typename ApproxMVBB::MyMatrix< _PREC_ >::template VectorStat<M>; \
   \
   template<typename Derived> using MatrixBase = typename ApproxMVBB::MyMatrix< _PREC_ >::template MatrixBase<Derived>; \
   template<typename Derived> using MatrixDenseBase = typename ApproxMVBB::MyMatrix< _PREC_ >::template MatrixDenseBase<Derived>; \
   \
   template<typename EigenType> using MatrixRef = typename ApproxMVBB::MyMatrix< _PREC_ >::template MatrixRef< EigenType >; \
   template<typename EigenType> using MatrixMap = typename ApproxMVBB::MyMatrix< _PREC_ >::template MatrixMap< EigenType >; \
   template<typename EigenType> using StdVecAligned = typename ApproxMVBB::MyMatrix< _PREC_ >::template StdVecAligned< EigenType >; \
   template<typename Key, typename EigenType> \
   using StdMapAligned = typename ApproxMVBB::MyMatrix< _PREC_ >::template StdMapAligned<Key, EigenType >; \
   \
   template<typename Derived> using ArrayBase  = typename ApproxMVBB::MyMatrix< _PREC_ >::template ArrayBase<Derived>; \
   template<unsigned int M> using ArrayStatDyn = typename ApproxMVBB::MyMatrix< _PREC_ >::template ArrayStatDyn<M>; \
   template<unsigned int N> using ArrayDynStat = typename ApproxMVBB::MyMatrix< _PREC_ >::template ArrayDynStat<N>; \
   template<unsigned int M,unsigned int N> using ArrayStatStat = typename ApproxMVBB::MyMatrix< _PREC_ >::template ArrayStatStat<M,N>; \
   template<unsigned int M> using ArrayStat = typename ApproxMVBB::MyMatrix< _PREC_ >::template ArrayStat<M>; \
   using Array3 = typename ApproxMVBB::MyMatrix< _PREC_ >::Array3;   \
   using Array2 = typename ApproxMVBB::MyMatrix< _PREC_ >::Array2; \
   \
   using AffineTrafo = typename ApproxMVBB::MyMatrix< _PREC_ >::AffineTrafo; \
   using AffineTrafo2d = typename ApproxMVBB::MyMatrix< _PREC_ >::AffineTrafo2d;

#endif

