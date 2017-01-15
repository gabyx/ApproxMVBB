// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include "ApproxMVBB/Diameter/Utils/rand.hpp"

namespace ApproxMVBB{

// 
// static long int _random_calls_ = 0;
// static long int _random_seed_ = 0;
// 
// long int _GetRandomCalls()
// {
//   return( _random_calls_ );
// }
// 
// long int _GetRandomSeed()
// {
//   return( _random_seed_ );
// }
// 
// 
// 
// #if defined _WIN32 || defined __CYGWIN__ || defined WIN32 // WIN32 is for legacy CMake
// void _SetRandomSeed( unsigned int seed )
// {
//   srand( seed );
//   _random_seed_ = seed;
//   _random_calls_ = 0;
// }
// #else
// void _SetRandomSeed( long int seed )
// {
//   srand48( seed );
//   //_random_seed_ = seed;
//   _random_calls_ = 0;
// }
// #endif
// 
// 
// 
// #if defined _WIN32 || defined __CYGWIN__ || defined WIN32	// WIN32 is for legacy CMake
// double _GetRandomDoubleNb( )
// {
//   //_random_calls_ ++;
//   return( (double)rand()/(double)RAND_MAX );
// }
// #else
// double _GetRandomDoubleNb( )
//{
  //_random_calls_ ++;
//   return( drand48() );
// }
// #endif
// 
// 
// int _GetRandomIntNb( int min, int max )
// {
//   if ( min <= max )
//     return( (int)(floor( min + _GetRandomDoubleNb()*(double)(max-min+1.0) )) );
//   return( (int)(floor( max + _GetRandomDoubleNb()*(double)(min-max+1.0) )) );
// }

}
