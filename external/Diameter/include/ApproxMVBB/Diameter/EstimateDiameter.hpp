// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================
#ifndef ApproxMVBB_Diameter_EstimateDiameter_hpp
#define ApproxMVBB_Diameter_EstimateDiameter_hpp

#include "ApproxMVBB/Diameter/TypeSegment.hpp"
#include "ApproxMVBB/RandomGenerators.hpp"

#include <random>

namespace ApproxMVBB{

/** Diameter Estimator class
*   If you need a longer period (currently 2^128-1) of the RandomGenerator
*   use the  RandomGenerators::XorShift1024Star
*/
class APPROXMVBB_EXPORT DiameterEstimator{
    public:

    DiameterEstimator(std::size_t seed = RandomGenerators::defaultSeed) : m_gen(seed) {}

    /** Raw function to estimate the diameter of a point cloud
    *   @param theDiam returns the diameter info
    *   @param theList input point cloud (look at example how to call this function)
    *   @param first First index into the point cloud
    *   @param dim dimension of the points
    *   @param epsilon the accuracy of the diameter estimation such that
               [diam_min, diam_max] <= epsilon, and [diam_min, diam_max] contains the true
               diameter

    */
    double estimateDiameter(Diameter::TypeSegment *theDiam,
                          double const**theList,
                          const int first,
                          const int last,
                          const int dim,
                          double epsilon);

    private:

    double estimateDiameterInOneList(Diameter::TypeSegment *theDiam,
                                     double const**theList,
                                     const int first,
                                     const int last,
                                     const int dim,
                                     double _epsilon_ );


    /** some settings from the original code ================================*/
    /** verbose when reducing*/
    int _verbose_when_reducing_ = 0;
    inline void _VerboseWhenReducing(){ _verbose_when_reducing_ = 1; }
    inline void _NoVerboseWhenReducing(){ _verbose_when_reducing_ = 0;}
    inline int _GetVerboseWhenReducing(){ return _verbose_when_reducing_ ;}


    /** reduction in the iterative search of the double normal */
    int _reduction_mode_in_iterative_ = 1;
    inline void _SetReductionModeInIterative( int m )
    {
      switch ( m ) {
      case 0 :
      case 1 :
        _reduction_mode_in_iterative_ = m;
        break;
      case 2 :
      default :
        break;
      }
    }
    inline int _GetReductionModeInIterative(){return _reduction_mode_in_iterative_ ;}

    /** 'reduction' of diameter */
    int _reduction_mode_of_diameter_ = 1;
    inline void _SetReductionModeOfDiameter( int m )
    {
      switch ( m ) {
      case 0 :
      case 1 :
      case 2 :
        _reduction_mode_of_diameter_ = m;
        break;
      default :
        break;
      }
    }
    int _GetReductionModeOfDiameter(){return _reduction_mode_of_diameter_; }

    /* 'reduction' of double normals
     */
    int _reduction_mode_of_dbleNorm_ = 1;
    inline void _SetReductionModeOfDbleNorm( int m )
    {
      switch ( m ) {
      case 0 :
      case 1 :
      case 2 :
        _reduction_mode_of_dbleNorm_ = m;
        break;
      default :
        break;
      }
    }
    inline int _GetReductionModeOfDbleNorm(){return _reduction_mode_of_dbleNorm_ ;}

    /* reduction by processing couple of double normals
     */
    int _try_to_reduce_Q_ = 1;

    inline void _DoTryToReduceQ() { _try_to_reduce_Q_ = 1;}
    inline void _DoNotTryToReduceQ(){ _try_to_reduce_Q_ = 0;}
    inline int _GetTryToReduceQ(){return _try_to_reduce_Q_ ; }


    int _Q_scan_ = 0;
    inline void _SetQscanToForward(){ _Q_scan_ = 1;}
    inline void _SetQscanToBackward(){ _Q_scan_ = 0;}
    inline int _GetQscan(){return _Q_scan_ ;}


    int _tight_bounds_ = 0;
    inline void _DoTryToGetTightBounds(){ _tight_bounds_ = 1;}
    inline void _DoNotTryToGetTightBounds(){ _tight_bounds_ = 0;}
    inline int _GetTightBounds(){ return _tight_bounds_ ; }
    /**=====================================================================*/

    RandomGenerators::DefaultRandomGen m_gen; ///< Random number generator
    
    /** TODO: ugly cast, I dont want to change the estimater code */
    int getRandomInt( unsigned int min, unsigned int max ){
      if ( min <= max ){
        return static_cast<int>(RandomGenerators::DefaultUniformUIntDistribution<unsigned int>{min,max}(m_gen));
      }else{
        return static_cast<int>(RandomGenerators::DefaultUniformUIntDistribution<unsigned int>{max,min}(m_gen));
      }
    }

};
}

#endif
