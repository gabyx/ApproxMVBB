// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Common_Platform_hpp
#define ApproxMVBB_Common_Platform_hpp

#include "ApproxMVBB/Config/Config.hpp"

#ifdef __CYGWIN__
  #include "ApproxMVBB/Common/CygwinPatch.hpp"
#endif

namespace ApproxMVBB{

#if (defined _WIN32) || (defined __CYGWIN__) || (defined WIN32)

  // This macro is given to the compiler when building the library!
  #ifdef ApproxMVBB_BUILD_LIBRARY

    #pragma message(" Platform.hpp: Building library ...")

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

  #ifdef ApproxMVBB_BUILD_LIBRARY

    #pragma message(" Platform.hpp: Building library ...")

    #if __GNUC__ >= 4 ||  __clang__
      #define APPROXMVBB_EXPORT __attribute__ ((visibility ("default")))
    #else
      #define APPROXMVBB_EXPORT
      #warning "Unknown compiler: Exporting everything into library!"
    #endif

  #else
     #define APPROXMVBB_EXPORT
  #endif

#endif

}

#endif
