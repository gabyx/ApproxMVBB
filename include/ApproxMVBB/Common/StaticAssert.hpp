// ========================================================================================
//  ApproxMVBB 
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//  
//  Licensed under GNU General Public License 3.0 or later. 
//  Some rights reserved. See COPYING, README.rst.
//
//  @license GPL-3.0 <http://www.gnu.org/licenses/gpl-3.0.html>
// ========================================================================================

#ifndef MVBB_Common_StaticAssert_hpp
#define MVBB_Common_StaticAssert_hpp


#include <type_traits>

namespace ApproxMVBB{
	#define ApproxMVBB_STATIC_ASSERT(B) static_assert( B , "no message");
	#define ApproxMVBB_STATIC_ASSERTM(B,COMMENT) static_assert( B , COMMENT);
};



#endif
