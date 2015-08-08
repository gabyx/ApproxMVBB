// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Common_Exception_hpp
#define ApproxMVBB_Common_Exception_hpp

#include <stdexcept>
#include <exception>
#include <string>
#include <sstream>

namespace ApproxMVBB{
    class Exception : public std::runtime_error {
    public:
        Exception(const std::stringstream & ss): std::runtime_error(ss.str()){}
    private:

    };
}

#define ApproxMVBB_THROWEXCEPTION( message ) {std::stringstream ___s___ ; ___s___ << message << std::endl << " @ " << __FILE__ << " (" << __LINE__ << ")" << std::endl; throw ApproxMVBB::Exception(___s___);}



#endif // Exception_hpp
