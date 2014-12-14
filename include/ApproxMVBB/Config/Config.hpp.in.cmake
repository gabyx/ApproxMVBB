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
// currently there are none!


namespace ApproxMVBB{

    static const unsigned int VersionMajor =  @ApproxMVBB_VERSION_MAJOR@ ;
    static const unsigned int VersionMinor =  @ApproxMVBB_VERSION_MINOR@ ;
    static const unsigned int VersionPatch =  @ApproxMVBB_VERSION_PATCH@ ;
    
    #define ApproxMVBB_AssertionDebug_INCLUDE_FILE          @ApproxMVBB_AssertionDebug_INCLUDE_FILE@
    #define ApproxMVBB_Exception_INCLUDE_FILE               @ApproxMVBB_Exception_INCLUDE_FILE@
    #define ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE        @ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE@
    #define ApproxMVBB_Platform_INCLUDE_FILE                @ApproxMVBB_Platform_INCLUDE_FILE@
    #define ApproxMVBB_StaticAssert_INCLUDE_FILE            @ApproxMVBB_StaticAssert_INCLUDE_FILE@
    #define ApproxMVBB_TypeDefs_INCLUDE_FILE                @ApproxMVBB_TypeDefs_INCLUDE_FILE@
    
};


#endif
