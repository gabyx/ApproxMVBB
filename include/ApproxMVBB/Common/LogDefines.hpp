// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Common_LogDefines_hpp
#define ApproxMVBB_Common_LogDefines_hpp

#include <iostream>

#include "ApproxMVBB/Config/Config.hpp"

#define ApproxMVBB_LOG(message )  { std::cout << message ; }
#define ApproxMVBB_LOGLEVEL(level,setlevel,message) { if( level <= setlevel ){  ApproxMVBB_LOG(message); } }
#define ApproxMVBB_LOGLEVEL_SCOPE(level) { if( level <= setlevel ) { }
#define ApproxMVBB_LOGLEVEL_SCOPE_END { } }

// Message Log ==================================================================================
#if ApproxMVBB_FORCE_MSGLOG_LEVEL > 0
    #define ApproxMVBB_MSGLOG_LEVEL ApproxMVBB_FORCE_MSGLOG_LEVEL // force the output level if set in the config!
#else
    #ifndef NDEBUG
    // Debug!
        #define ApproxMVBB_MSGLOG_LEVEL 2 // 0 = no output
    #else
        #define ApproxMVBB_MSGLOG_LEVEL 0 // 0 = no output
    #endif
#endif

#define ApproxMVBB_MSGLOG_L1(message )  { ApproxMVBB_LOGLEVEL(1, ApproxMVBB_MSGLOG_LEVEL, message) }
#define ApproxMVBB_MSGLOG_L2(message )  { ApproxMVBB_LOGLEVEL(2, ApproxMVBB_MSGLOG_LEVEL, message) }
#define ApproxMVBB_MSGLOG_L3(message )  { ApproxMVBB_LOGLEVEL(3, ApproxMVBB_MSGLOG_LEVEL, message) }
// ==============================================================================================

#endif
