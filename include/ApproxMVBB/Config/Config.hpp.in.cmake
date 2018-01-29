// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Config_Config_hpp_in_cmake
#define ApproxMVBB_Config_Config_hpp_in_cmake

// Settings for the ApproxMVBB Library


namespace ApproxMVBB{

    static const unsigned int VersionMajor =  @ApproxMVBB_VERSION_MAJOR@ ;
    static const unsigned int VersionMinor =  @ApproxMVBB_VERSION_MINOR@ ;
    static const unsigned int VersionPatch =  @ApproxMVBB_VERSION_PATCH@ ;
    
    // This variable is primarily used for projects which used this source code but want to replace certain header files with their own!
    #cmakedefine ApproxMVBB_USE_DIFFERENT_HEADERS
    
    #ifdef ApproxMVBB_USE_DIFFERENT_HEADERS
        #define ApproxMVBB_AssertionDebug_INCLUDE_FILE          "@ApproxMVBB_AssertionDebug_INCLUDE_FILE@"
        #define ApproxMVBB_Exception_INCLUDE_FILE               "@ApproxMVBB_Exception_INCLUDE_FILE@"
        #define ApproxMVBB_Platform_INCLUDE_FILE                "@ApproxMVBB_Platform_INCLUDE_FILE@"
        #define ApproxMVBB_StaticAssert_INCLUDE_FILE            "@ApproxMVBB_StaticAssert_INCLUDE_FILE@"
        
        #define ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE        "@ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE@"
        #define ApproxMVBB_MyContainerTypeDefs_INCLUDE_FILE     "@ApproxMVBB_MyContainerTypeDefs_INCLUDE_FILE@"
        
        #define ApproxMVBB_TypeDefs_INCLUDE_FILE                "@ApproxMVBB_TypeDefs_INCLUDE_FILE@"
        #define ApproxMVBB_AABB_INCLUDE_FILE                    "@ApproxMVBB_AABB_INCLUDE_FILE@"
        #define ApproxMVBB_OOBB_INCLUDE_FILE                    "@ApproxMVBB_OOBB_INCLUDE_FILE@"
        
    #else
        #define ApproxMVBB_AssertionDebug_INCLUDE_FILE          "ApproxMVBB/Common/AssertionDebug.hpp"
        #define ApproxMVBB_Exception_INCLUDE_FILE               "ApproxMVBB/Common/Exception.hpp"
        
        #define ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE        "ApproxMVBB/Common/MyMatrixTypeDefs.hpp"
        #define ApproxMVBB_MyContainerTypeDefs_INCLUDE_FILE     "ApproxMVBB/Common/MyContainerTypeDefs.hpp"

        #define ApproxMVBB_Platform_INCLUDE_FILE                "ApproxMVBB/Common/Platform.hpp"
        #define ApproxMVBB_StaticAssert_INCLUDE_FILE            "ApproxMVBB/Common/StaticAssert.hpp"
        #define ApproxMVBB_TypeDefs_INCLUDE_FILE                "ApproxMVBB/Common/TypeDefs.hpp"
        #define ApproxMVBB_AABB_INCLUDE_FILE                    "ApproxMVBB/AABB.hpp"
        #define ApproxMVBB_OOBB_INCLUDE_FILE                    "ApproxMVBB/OOBB.hpp" 
        
    #endif
    
    // Force log level
    #define ApproxMVBB_FORCE_MSGLOG_LEVEL @ApproxMVBB_FORCE_MSGLOG_LEVEL@
    
    #cmakedefine ApproxMVBB_OPENMP_SUPPORT 
    #ifdef ApproxMVBB_OPENMP_SUPPORT
    
        #define ApproxMVBB_OPENMP_NTHREADS @ApproxMVBB_OPENMP_NTHREADS@
        
        #cmakedefine ApproxMVBB_OPENMP_USE_NTHREADS
        #ifdef ApproxMVBB_OPENMP_USE_NTHREADS
            #define ApproxMVBB_OPENMP_NUMTHREADS num_threads(ApproxMVBB_OPENMP_NTHREADS)
        #else
            #define ApproxMVBB_OPENMP_NUMTHREADS
        #endif
    #endif

}


#endif
