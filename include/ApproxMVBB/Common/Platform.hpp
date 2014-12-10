// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
// ========================================================================================

#ifndef ApproxMVBB_Common_Platform_hpp
#define ApproxMVBB_Common_Platform_hpp

#include "ApproxMVBB/Config/Config.hpp"

namespace ApproxMVBB{

#if defined _WIN32 || defined __CYGWIN__

  #if ApproxMVBB_BUILD_LIBRARY

    #ifdef __GNUC__
      #define APPROXMVBB_EXPORT __attribute__ ((dllexport))
    #else
      #define APPROXMVBB_EXPORT __declspec(dllexport) // Note: actually gcc seems to also supports this syntax.
    #endif

  #else

    #ifdef __GNUC__
      #define APPROXMVBB_EXPORT __attribute__ ((dllimport))
    #else
      #define APPROXMVBB_EXPORT __declspec(dllimport) // Note: actually gcc seems to also supports this syntax.
    #endif

  #endif

#else

  #if __GNUC__ >= 4 ||  __clang__
    #define APPROXMVBB_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define APPROXMVBB_EXPORT
    #warning "Unknown compiler: Exporting everything into library!"
  #endif

#endif

};

#endif
