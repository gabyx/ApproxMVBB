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

#include ApproxMVBB_Exception_INCLUDE_FILE

#ifndef NDEBUG
// Debug!
	/**
	* @brief An Assert Macro to use within C++ code.
	* @param condition The condition which needs to be truem otherwise an assertion is thrown!
	*/
    #define ASSERTMSG(condition , message) { if(!(condition)){ ERRORMSG(message) } }
    #define WARNINGMSG(condition , message) { if(!(condition)){ ERRORMSG(message) } }
#else
   #define ASSERTMSG(condition,message)
   #define WARNINGMSG(condition,message)
#endif

   #define ERRORMSG( message ) THROWEXCEPTION( message )

#endif
