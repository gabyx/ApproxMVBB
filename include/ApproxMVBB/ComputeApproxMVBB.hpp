// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_ComputeApproxMVBB_hpp
#define ApproxMVBB_ComputeApproxMVBB_hpp

#include <array>

#include "ApproxMVBB/Common/LogDefines.hpp"
#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE
#include "ApproxMVBB/Common/TypeDefs.hpp"
#include ApproxMVBB_OOBB_INCLUDE_FILE
#include "ApproxMVBB/GreatestCommonDivisor.hpp"
#include "ApproxMVBB/ProjectedPointSet.hpp"
#include "ApproxMVBB/RandomGenerators.hpp"

namespace ApproxMVBB
{
    ApproxMVBB_DEFINE_MATRIX_TYPES;
    ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES;

    /*!
        We are given a point set, and (hopefully) a tight fitting
        bounding box. We compute a sample of the given size nPoints that
        represents the point-set. The only guarenteed is that if we use sample of size m,
        we get an approximation of quality about 1/\sqrt{m}. Note that we pad
        the sample if necessary to get the desired size.
        This function changes the oobb and sets the z Axis to the greatest
        extent!
        @param nPoints needs to be greater or equal than 2
    */
    template<typename Derived>
    void samplePointsGrid(Matrix3Dyn& newPoints,
                          const MatrixBase<Derived>& points,
                          const unsigned int nPoints,
                          OOBB& oobb,
                          std::size_t seed = ApproxMVBB::RandomGenerators::defaultSeed)
    {
        using IndexType = typename Derived::Index;

        if(nPoints > points.cols() || nPoints < 2)
        {
            ApproxMVBB_ERRORMSG("Wrong arguments!"
                                << "sample nPoints: (>2) " << nPoints << " of points: " << points.cols() << std::endl)
        }

        newPoints.resize(3, nPoints);

        // total points = bottomPoints=gridSize^2  + topPoints=gridSize^2
        unsigned int gridSize = std::max(static_cast<unsigned int>(std::sqrt(static_cast<double>(nPoints) / 2.0)), 1U);

        // Set z-Axis to longest dimension
        // std::cout << oobb.m_minPoint.transpose() << std::endl;
        oobb.setZAxisLongest();

        IndexType halfSampleSize = gridSize * gridSize;

        struct BottomTopPoints
        {
            IndexType bottomIdx = 0;
            PREC bottomZ;

            IndexType topIdx = 0;
            PREC topZ;
        };

        // grid of the bottom/top points in Z direction (indexed from 1 )
        std::vector<BottomTopPoints> boundaryPoints(halfSampleSize);

        using LongInt = long long int;
        MyMatrix::Array2<LongInt> idx;  // Normalized P
        // std::cout << oobb.extent() << std::endl;
        // std::cout << oobb.m_minPoint.transpose() << std::endl;
        Array2 dxdyInv = Array2(gridSize, gridSize) / oobb.extent().head<2>();  // in K Frame;
        Vector3 K_p;

        Matrix33 A_KI(oobb.m_q_KI.matrix().transpose());

        // Register points in grid
        IndexType size = points.cols();
        for(IndexType i = 0; i < size; ++i)
        {
            K_p = A_KI * points.col(i);
            // get x index in grid
            idx = ((K_p - oobb.m_minPoint).head<2>().array() * dxdyInv).template cast<LongInt>();
            // map to grid
            idx(0) = std::max(std::min(LongInt(gridSize - 1), idx(0)), 0LL);
            idx(1) = std::max(std::min(LongInt(gridSize - 1), idx(1)), 0LL);
            // std::cout << idx.transpose() << std::endl;

            // Register points in grid
            // if z component of p is > pB.topZ  -> set new top point at pos
            // if z component of p is < pB.bottomZ    -> set new bottom point at pos
            auto& pB = boundaryPoints[idx(0) + idx(1) * gridSize];

            if(pB.bottomIdx == 0)
            {
                pB.bottomIdx = pB.topIdx = i + 1;
                pB.bottomZ = pB.topZ = K_p(2);
            }
            else
            {
                if(pB.topZ < K_p(2))
                {
                    pB.topIdx = i + 1;
                    pB.topZ   = K_p(2);
                }
                else
                {
                    if(pB.bottomZ > K_p(2))
                    {
                        pB.bottomIdx = i + 1;
                        pB.bottomZ   = K_p(2);
                    }
                }
            }
        }

        // Copy top and bottom points
        IndexType k = 0;
        ApproxMVBB_MSGLOG_L2("Sampled Points incides: [ ");
            // k does not overflow -> 2* halfSampleSize = 2*gridSize*gridSize <=
            // nPoints;
            for(IndexType i = 0; i < halfSampleSize; ++i)
        {
            if(boundaryPoints[i].bottomIdx != 0)
            {
                // comment in if you want the top/bottom points of the grid
                // Array3 a(i %
                // gridSize,i/gridSize,oobb.m_maxPoint(2)-oobb.m_minPoint(2));
                // a.head<2>()*=dxdyInv.inverse();
                ApproxMVBB_MSGLOG_L2(boundaryPoints[i].topIdx - 1 << ", " << ((k % 30 == 0) ? "\n" : ""));
                newPoints.col(k++) = points.col(boundaryPoints[i].topIdx - 1);  //  A_KI.transpose()*(oobb.m_minPoint + a.matrix()).eval() ;
                if(boundaryPoints[i].topIdx != boundaryPoints[i].bottomIdx)
                {
                    // comment in if you want the bottom points of the grid
                    // Array3 a(i % gridSize,i/gridSize,0);
                    // a.head<2>()*=dxdyInv.inverse();
                    ApproxMVBB_MSGLOG_L2(boundaryPoints[i].bottomIdx - 1 << ", ");
                    newPoints.col(k++) = points.col(boundaryPoints[i].bottomIdx - 1);  //  A_KI.transpose()*(oobb.m_minPoint + a.matrix()).eval() ;
                }
            }
        }

        // Add random points!
        // Random indices if too little points
        if(k < nPoints)
        {
            RandomGenerators::DefaultRandomGen gen(seed);
            RandomGenerators::DefaultUniformUIntDistribution<typename std::make_unsigned<IndexType>::type> dis(
                0, points.cols() - 1);
            IndexType s;
            while(k < nPoints)
            {
                s = dis(gen);
                ApproxMVBB_MSGLOG_L2(s << ", ");
                newPoints.col(k++) = points.col(s);  //= Vector3(0,0,0);//
            }
        }
        ApproxMVBB_MSGLOG_L2("]" << std::endl);
    }

    /*!
        Function to optimize oriented bounding box volume.
        Projecting nLoops times into the direction of the axis of the current oobb,
        constructing the mvbb and overwriting the current oobb if volume is smaller
        @param volumeAcceptFactor is volumeAcceptTol = oobb.volume *
        volumeAcceptFactor, which determines the tolerance when a new volume is
        accepted
        @param minBoxExtent is the minmum extent direction a box must have, to make
        the volume not zero and comparable to other volumes
        which is useful for degenerated cases, such as all points in a surface 
    */
    template<typename Derived>
    OOBB optimizeMVBB(const MatrixBase<Derived>& points,
                      OOBB oobb,
                      unsigned int nLoops     = 10,
                      PREC volumeAcceptFactor = 1e-6,
                      PREC minBoxExtent       = 1e-12)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, Eigen::Dynamic);

        if(oobb.volume() == 0.0 || nLoops == 0)
        {
            return oobb;
        }

        // Define the volume lower bound above we accept a new volume as
        PREC volumeAcceptTol = oobb.volume() * volumeAcceptFactor;

        //    // Parallel Loop Start
        //    ===============================================================
        //    #ifdef ApproxMVBB_OPENMP_SUPPORT
        //        #pragma omp declare reduction(volumeIsSmaller:OOBB: \
        //                                            omp_in.volume()<omp_out.volume() ? omp_out=omp_in : omp_out \
        //                                     ) initializer(omp_priv(omp_orig))
        //
        //        #pragma omp parallel for schedule(dynamic,1) \
        //                                 shared(points) \
        //                                 reduction(volumeIsSmaller:oobb) \
        //                                 ApproxMVBB_OPENMP_NUMTHREADS
        //
        //
        //        for(unsigned int parallelIdx = 0; parallelIdx < 12 ; ++parallelIdx )
        //        {
        //
        //    #endif

        unsigned int cacheIdx = 0;        // current write Idx into the cache
        std::array<Vector3, 3> dirCache;  // save the last three directions (avoiding
                                          // cycling in choosen axis)

        Vector3 dir;
        ProjectedPointSet proj;
        for(unsigned int loop = 0; loop < nLoops; ++loop)
        {
            // Determine Direction (choose x or y axis)
            // std::cout << oobb.m_q_KI.matrix() << std::endl;
            dir = oobb.getDirection(0);

            // check against all cache values
            for(unsigned char i = 0; i < 3 && i < loop; ++i)
            {
                PREC dotp = std::abs(dir.dot(dirCache[i]));  //
                if(std::abs(dotp - 1.0) <= 1e-3)
                {
                    // std::cout << "Change Dir" << std::endl;
                    // direction are almost the same as in the cache, choose another one
                    dir = oobb.getDirection(1);
                    break;
                }
            }
            // Write to cache and shift write idx
            dirCache[cacheIdx] = dir;
            cacheIdx           = (cacheIdx + 1) % 3;

            // std::cout << "Optimizing dir: " << dir << std::endl;
            OOBB o = proj.computeMVBB(dir, points);

            // Expand box such the volume is not zero for points in a plane
            o.expandToMinExtentAbsolute(minBoxExtent);

            if(o.volume() < oobb.volume() && o.volume() > volumeAcceptTol)
            {
                oobb = o;
            }
        }

        //    // Parallell Loop End
        //    =====================================================================
        //    #ifdef ApproxMVBB_OPENMP_SUPPORT
        //        }
        //    #endif

        return oobb;
    }

    /*!
        Function to optimize oriented bounding box volume.
        This performs an exhaustive grid search over a given tighly fitted bounding
        box (use approximateMVBBDiam)
        to find a tighter volume.
        
        @param gridSize is half the grid size of the 3d test grid in each direction,
        for example gridDimX , gridDimY, gridDimZ = [-gridSize,gridSize]
        @param optLoops how many optimization loops are preformed for the oobb computed 
        in the given discrete sampled direction in the
        grid  (see optimizeMVBB)
        @param volumeAcceptFactor is volumeAcceptTol = oobb.volume *
        volumeAcceptFactor, which determines the tolerance when a new volume is
        accepted
        @param minBoxExtent is the minmum extent direction a box must have, to make
        the volume not zero and comparable to other volumes
        which is useful for degenerate cases, such as all points in a surface
    */
    template<typename Derived>
    OOBB approximateMVBBGridSearch(const MatrixBase<Derived>& points,
                                   OOBB oobb,
                                   PREC epsilon,
                                   const unsigned int gridSize = 5,
                                   const unsigned int optLoops = 6,
                                   PREC volumeAcceptFactor     = 1e-6,
                                   PREC minBoxExtent           = 1e-12)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, Eigen::Dynamic);

        // Extent input oobb
        oobb.expandToMinExtentAbsolute(minBoxExtent);

        // Define the volume lower bound above we accept a new volume as
        // PREC volumeAcceptTol = oobb.volume() * volumeAcceptFactor;

        // Get the direction of the input OOBB in coordinate system `I` :
        Vector3 dir1 = oobb.getDirection(0);
        Vector3 dir2 = oobb.getDirection(1);
        Vector3 dir3 = oobb.getDirection(2);

        ProjectedPointSet proj;
        Vector3 dir;  // clang-format off
        
#ifdef ApproxMVBB_OPENMP_SUPPORT
    #if _OPENMP <= 200203
        #pragma omp parallel shared(points, oobb) private(proj, dir)
        {
            OOBB oobbLocal = oobb;
            // Redirect the variable `oobb` to the local one.
            #define oobb oobbLocal 
            #pragma omp for
    #else
        #pragma omp declare reduction(volumeIsSmaller                                              \
                                : OOBB                                                             \
                                : omp_in.volume() < omp_out.volume() ? omp_out = omp_in : omp_out) \
        initializer(omp_priv(omp_orig))

        #pragma omp parallel for schedule(dynamic, 4) collapse(3) shared(points) private(proj, dir) \
                reduction(volumeIsSmaller                                                           \
                        : oobb) ApproxMVBB_OPENMP_NUMTHREADS
    #endif
#endif
        // clang-format on
        for(int x = -int(gridSize); x <= (int)gridSize; ++x)
        {
            for(int y = -int(gridSize); y <= (int)gridSize; ++y)
            {
                for(int z = 0; z <= (int)gridSize; ++z)
                {
                    if(MathFunctions::gcd3(x, y, z) > 1 || ((x == 0) && (y == 0) && (z == 0)))
                    {
                        continue;
                    }

                    // Make direction
                    Vector3 dir = x * dir1 + y * dir2 + z * dir3;
                    ApproxMVBB_MSGLOG_L3("gridSearch: dir: " << dir.transpose() << std::endl);

                    // Compute MVBB in dirZ
                    auto res = proj.computeMVBB(dir, points);

                    // Expand to minimal extent for points in a surface or line
                    res.expandToMinExtentAbsolute(minBoxExtent);

                    if(optLoops)
                    {
                        res = optimizeMVBB(points, res, optLoops, volumeAcceptFactor, minBoxExtent);
                    }
                    ApproxMVBB_MSGLOG_L3("gridSearch: volume: "
                                         << res.volume()
                                         << std::endl);

                    if(res.volume() < oobb.volume() /*&& res.volume()>volumeAcceptTol */)
                    {
                        ApproxMVBB_MSGLOG_L2("gridSearch: new volume: " << res.volume() << std::endl
                                                                        << "for dir: " << dir.transpose() << std::endl);
                        oobb = res;
                    }
                }
            }
        }
        
#if defined(ApproxMVBB_OPENMP_SUPPORT) && _OPENMP <= 200203
            #undef oobb
            #pragma omp critical
            if (oobbLocal.volume() < oobb.volume())
            {
                oobb = oobbLocal;
            }
        }
#endif

        return oobb;
    }

    /*!
        Function to optimize oriented bounding box volume.
        This constructs an approximation of a tightly fitted bounding box by computing
        the diameter d in 3d and afterwards the projection of the points in the plane
        perpendicular to direction d
        and then the diameter f in 2d and extruding the OOBB in 2d to the final OOBB
        approximation in 3d.
    */
    template<typename Derived>
    OOBB approximateMVBBDiam(const MatrixBase<Derived>& points,
                             const PREC epsilon,
                             const unsigned int optLoops = 10,
                             std::size_t seed            = ApproxMVBB::RandomGenerators::defaultSeed)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, Eigen::Dynamic);

        using namespace PointFunctions;
        auto pp = estimateDiameter<3>(points, epsilon, seed);

        ApproxMVBB::MyMatrix::Vector3<ApproxMVBB::TypeDefsPoints::PREC> dirZ = pp.first - pp.second;

        // TODO: Is this direction inversion needed? not really ?
        if((dirZ.array() <= 0.0).all())
        {
            dirZ *= -1;
        }

        // If direction zero, use (1,0)
        if((dirZ.array() == 0.0).all())
        {
            dirZ.setZero();
            dirZ(0) = 1;
        }
        ApproxMVBB_MSGLOG_L1("estimated 3d diameter: " << dirZ.transpose() << " eps: " << epsilon << std::endl);

        // Compute MVBB in dirZ
        ProjectedPointSet proj;
        // OOBB oobb = proj.computeMVBB();
        // or faster estimate diameter in projected plane and build coordinate system
        OOBB oobb = proj.computeMVBBApprox(dirZ, points, epsilon);

        if(optLoops)
        {
            oobb = optimizeMVBB(points, oobb, optLoops);
        }
        return oobb;
    }

    template<typename Derived>
    OOBB approximateMVBB(const MatrixBase<Derived>& points,
                         const PREC epsilon,
                         const unsigned int pointSamples           = 400,
                         const unsigned int gridSize               = 5,
                         const unsigned int mvbbDiamOptLoops       = 0,
                         const unsigned int mvbbGridSearchOptLoops = 6,
                         std::size_t seed                          = ApproxMVBB::RandomGenerators::defaultSeed)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, Eigen::Dynamic);

        // Get get MVBB from estimated diameter direction
        // take care forwarding means not using gen anymore !
        auto oobb = approximateMVBBDiam(points, epsilon, mvbbDiamOptLoops, seed);

        // Check if we sample the point cloud
        if(pointSamples < points.cols())
        {
            // sample points
            Matrix3Dyn sampled;
            samplePointsGrid(sampled, points, pointSamples, oobb, seed);

            // Exhaustive grid search with sampled points
            oobb = approximateMVBBGridSearch(sampled, oobb, epsilon, gridSize, mvbbGridSearchOptLoops);
        }
        else
        {
            // Exhaustive grid search with sampled points
            oobb = approximateMVBBGridSearch(points, oobb, epsilon, gridSize, mvbbGridSearchOptLoops);
        }

        return oobb;
    }
}  // namespace ApproxMVBB

#endif  // ApproxMVBB_hpp
