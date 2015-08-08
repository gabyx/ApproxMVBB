// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
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
#else
     #define ApproxMVBB_ASSERTMSG(condition , message) { if(!(condition)){ ApproxMVBB_ERRORMSG(message) } }
#endif

    #define ApproxMVBB_WARNINGMSG(condition , message) { if(!(condition)){ std::cerr << "WARNING: " << #condition << " : " <<std::endl<< message << std::endl << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl;} }
    #define ApproxMVBB_ERRORMSG( message ) ApproxMVBB_THROWEXCEPTION( message )

#endif
