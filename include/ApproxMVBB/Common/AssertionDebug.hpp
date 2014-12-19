// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  Licensed under GNU General Public License 3.0 or later.
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef ApproxMVBB_Common_AssertionDebug_hpp
#define ApproxMVBB_Common_AssertionDebug_hpp

#include <stdlib.h>
#include <iostream>
#include <typeinfo>

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_Exception_INCLUDE_FILE

#ifndef NDEBUG
// Debug!
	/**
	* @brief An Assert Macro to use within C++ code.
	* @param condition The condition which needs to be truem otherwise an assertion is thrown!
	*/
    #define ApproxMVBB_ASSERTMSG(condition , message) { if(!(condition)){ ApproxMVBB_ERRORMSG(message) } }
    #define ApproxMVBB_WARNINGMSG(condition , message) { if(!(condition)){ std::cerr << "WARNING: " <<message << std::endl; } }
#else
   #define ApproxMVBB_ASSERTMSG(condition,message)
   #define ApproxMVBB_WARNINGMSG(condition,message)
#endif

   #define ApproxMVBB_ERRORMSG( message ) ApproxMVBB_THROWEXCEPTION( message )

#endif
