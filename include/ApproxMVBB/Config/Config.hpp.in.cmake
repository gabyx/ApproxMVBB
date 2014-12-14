// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef ApproxMVBB_Config_Config_hpp
#define ApproxMVBB_Config_Config_hpp

// Settings for the ApproxMVBB Library


namespace ApproxMVBB{

    static const unsigned int VersionMajor =  @ApproxMVBB_VERSION_MAJOR@ ;
    static const unsigned int VersionMinor =  @ApproxMVBB_VERSION_MINOR@ ;
    static const unsigned int VersionPatch =  @ApproxMVBB_VERSION_PATCH@ ;
    
    // This variable is primarily used for projects which used this source code but want to replace certain header files with their own!
    #cmakedefine ApproxMVBB_USE_DIFFERENT_HEADERS
    
    #ifdef ApproxMVBB_USE_DIFFERENT_HEADERS
        #define ApproxMVBB_AssertionDebug_INCLUDE_FILE          @ApproxMVBB_AssertionDebug_INCLUDE_FILE@
        #define ApproxMVBB_Exception_INCLUDE_FILE               @ApproxMVBB_Exception_INCLUDE_FILE@
        #define ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE        @ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE@
        #define ApproxMVBB_Platform_INCLUDE_FILE                @ApproxMVBB_Platform_INCLUDE_FILE@
        #define ApproxMVBB_StaticAssert_INCLUDE_FILE            @ApproxMVBB_StaticAssert_INCLUDE_FILE@
        #define ApproxMVBB_TypeDefs_INCLUDE_FILE                @ApproxMVBB_TypeDefs_INCLUDE_FILE@
        #define ApproxMVBB_AABB_INCLUDE_FILE                    @ApproxMVBB_AABB_INCLUDE_FILE@
        #define ApproxMVBB_OOBB_INCLUDE_FILE                    @ApproxMVBB_OOBB_INCLUDE_FILE@
    #else
        #define ApproxMVBB_AssertionDebug_INCLUDE_FILE          "ApproxMVBB/Common/AssertionDebug.hpp"@
        #define ApproxMVBB_Exception_INCLUDE_FILE               "ApproxMVBB/Common/Exception.hpp"
        #define ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE        "ApproxMVBB/Common/MyMatrixTypeDefs.hpp"
        #define ApproxMVBB_Platform_INCLUDE_FILE                "ApproxMVBB/Common/Platform.hpp"
        #define ApproxMVBB_StaticAssert_INCLUDE_FILE            "ApproxMVBB/Common/StaticAssert.hpp"
        #define ApproxMVBB_TypeDefs_INCLUDE_FILE                "ApproxMVBB/Common/TypeDefs.hpp"
        #define ApproxMVBB_AABB_INCLUDE_FILE                    "ApproxMVBB/AABB.hpp"
        #define ApproxMVBB_OOBB_INCLUDE_FILE                    "ApproxMVBB/OOBB.hpp" 
    #endif
    
    
};


#endif
